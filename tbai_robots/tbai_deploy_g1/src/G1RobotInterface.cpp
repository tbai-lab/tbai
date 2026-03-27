#include "tbai_deploy_g1/G1RobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

void G1RobotInterface::initMotorMapping() {
    motorIdMap_["left_hip_pitch_joint"] = 0;
    motorIdMap_["left_hip_roll_joint"] = 1;
    motorIdMap_["left_hip_yaw_joint"] = 2;
    motorIdMap_["left_knee_joint"] = 3;
    motorIdMap_["left_ankle_pitch_joint"] = 4;
    motorIdMap_["left_ankle_roll_joint"] = 5;
    motorIdMap_["right_hip_pitch_joint"] = 6;
    motorIdMap_["right_hip_roll_joint"] = 7;
    motorIdMap_["right_hip_yaw_joint"] = 8;
    motorIdMap_["right_knee_joint"] = 9;
    motorIdMap_["right_ankle_pitch_joint"] = 10;
    motorIdMap_["right_ankle_roll_joint"] = 11;
    motorIdMap_["waist_yaw_joint"] = 12;
    motorIdMap_["waist_roll_joint"] = 13;
    motorIdMap_["waist_pitch_joint"] = 14;
    motorIdMap_["left_shoulder_pitch_joint"] = 15;
    motorIdMap_["left_shoulder_roll_joint"] = 16;
    motorIdMap_["left_shoulder_yaw_joint"] = 17;
    motorIdMap_["left_elbow_joint"] = 18;
    motorIdMap_["left_wrist_roll_joint"] = 19;
    motorIdMap_["left_wrist_pitch_joint"] = 20;
    motorIdMap_["left_wrist_yaw_joint"] = 21;
    motorIdMap_["right_shoulder_pitch_joint"] = 22;
    motorIdMap_["right_shoulder_roll_joint"] = 23;
    motorIdMap_["right_shoulder_yaw_joint"] = 24;
    motorIdMap_["right_elbow_joint"] = 25;
    motorIdMap_["right_wrist_roll_joint"] = 26;
    motorIdMap_["right_wrist_pitch_joint"] = 27;
    motorIdMap_["right_wrist_yaw_joint"] = 28;
}

G1RobotInterface::G1RobotInterface(G1RobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_g1");
    TBAI_LOG_INFO(logger_, "G1RobotInterface constructor (tbai_sdk/zenoh backend)");

    initMotorMapping();

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", G1_TOPIC_LOWCMD);
    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>(G1_TOPIC_LOWCMD);

    // Initialize InEKF state estimator with left and right foot frames
    std::vector<std::string> footNames = {"left_ankle_roll_link", "right_ankle_roll_link"};
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, "");
    TBAI_LOG_INFO(logger_, "InEKF estimator initialized");

    rectifyOrientation_ = tbai::fromGlobalConfig<bool>("inekf_estimator/rectify_orientation", true);
    removeGyroscopeBias_ = tbai::fromGlobalConfig<bool>("inekf_estimator/remove_gyroscope_bias", true);

    useGroundTruthState_ = args.useGroundTruthState();
    if (useGroundTruthState_) {
        TBAI_LOG_INFO(logger_, "Using ground-truth position/velocity from LowState (when available)");
    }

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", G1_TOPIC_LOWSTATE);
    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        G1_TOPIC_LOWSTATE, [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); }, 1);
}

G1RobotInterface::~G1RobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying G1RobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void G1RobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
    auto t11 = std::chrono::high_resolution_clock::now();
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    static auto last_time2 = currentTime;
    static int count = 0;
    count++;

    constexpr int N = 200;
    if (count % N == 0) {
        scalar_t time_diff = currentTime - last_time2;
        double rate = N / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "Low state callback rate: {} Hz (count: {})", rate, count);
        last_time2 = currentTime;
    }

    vector_t jointAngles(G1_NUM_JOINTS);
    vector_t jointVelocities(G1_NUM_JOINTS);

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        jointAngles[i] = low_state.motor_states[i].q;
        jointVelocities[i] = low_state.motor_states[i].dq;
    }

    vector4_t baseOrientation;
    baseOrientation[0] = low_state.imu_state.quaternion[1];  // x
    baseOrientation[1] = low_state.imu_state.quaternion[2];  // y
    baseOrientation[2] = low_state.imu_state.quaternion[3];  // z
    baseOrientation[3] = low_state.imu_state.quaternion[0];  // w

    vector3_t baseAngVel;
    baseAngVel[0] = low_state.imu_state.gyroscope[0];
    baseAngVel[1] = low_state.imu_state.gyroscope[1];
    baseAngVel[2] = low_state.imu_state.gyroscope[2];

    vector3_t baseAcc;
    baseAcc[0] = low_state.imu_state.accelerometer[0];
    baseAcc[1] = low_state.imu_state.accelerometer[1];
    baseAcc[2] = low_state.imu_state.accelerometer[2];

    // G1 bipedal contact detection with hysteresis
    // Bridge foot_force: FR(0)=right, FL(1)=left
    static std::vector<bool> prevContacts = {true, true};
    std::vector<bool> contactFlags(2, true);
    const double on_threshold = 10.0;
    const double off_threshold = 3.0;
    if (low_state.foot_force.size() >= 2) {
        double leftForce = static_cast<double>(low_state.foot_force[1]);
        double rightForce = static_cast<double>(low_state.foot_force[0]);
        contactFlags[0] = prevContacts[0] ? (leftForce >= off_threshold) : (leftForce >= on_threshold);
        contactFlags[1] = prevContacts[1] ? (rightForce >= off_threshold) : (rightForce >= on_threshold);
        prevContacts = contactFlags;
    }

    static scalar_t lastTime = currentTime;
    scalar_t dt = currentTime - lastTime;
    lastTime = currentTime;

    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags, rectifyOrientation_, enable_);

    State state;
    state.x = vector_t::Zero(G1_STATE_DIM);

    const quaternion_t baseQuaternion =
        rectifyOrientation_ ? quaternion_t(baseOrientation) : quaternion_t(estimator_->getBaseOrientation());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;

    if (useGroundTruthState_ && low_state.has_position && low_state.has_velocity) {
        vector3_t gtPos(low_state.position[0], low_state.position[1], low_state.position[2]);
        vector3_t gtVel(low_state.velocity[0], low_state.velocity[1], low_state.velocity[2]);
        state.x.segment<3>(3) = gtPos;
        state.x.segment<3>(9) = R_base_world * gtVel;
    } else {
        state.x.segment<3>(3) = estimator_->getBasePosition();
        state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();
    }

    if (removeGyroscopeBias_) {
        state.x.segment<3>(6) = baseAngVel - estimator_->getGyroscopeBias();
    } else {
        state.x.segment<3>(6) = baseAngVel;
    }
    state.x.segment(12, G1_NUM_JOINTS) = jointAngles;
    state.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    auto t12 = std::chrono::high_resolution_clock::now();
    TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "State update time: {} us",
                           std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count());

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    baseQuaternion_ = std::move(baseOrientation);
    initialized_ = true;
}

void G1RobotInterface::publish(std::vector<MotorCommand> commands) {
    static auto last_publish_time = std::chrono::high_resolution_clock::now();
    static int publish_count = 0;
    publish_count++;

    constexpr int PUBLISH_N = 100;
    if (publish_count % PUBLISH_N == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_publish_time).count();
        double rate = (PUBLISH_N * 1000.0) / time_diff;
        TBAI_LOG_INFO(logger_, "Publish frequency: {} Hz (count: {})", rate, publish_count);
        last_publish_time = current_time;
    }

    robot_msgs::MotorCommands motor_commands;
    motor_commands.commands.resize(G1_NUM_JOINTS);

    for (const auto &command : commands) {
        const int motorId = motorIdMap_[command.joint_name];
        motor_commands.commands[motorId].q = command.desired_position;
        motor_commands.commands[motorId].dq = command.desired_velocity;
        motor_commands.commands[motorId].kp = command.kp;
        motor_commands.commands[motorId].kd = command.kd;
        motor_commands.commands[motorId].tau = command.torque_ff;
    }

    lowcmd_publisher->publish(motor_commands);
}

void G1RobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        TBAI_LOG_INFO_THROTTLE(logger_, 3.0, "Waiting for the G1 robot to initialize...");
    }
    TBAI_LOG_DEBUG(logger_, "G1 Robot initialized");
}

State G1RobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
