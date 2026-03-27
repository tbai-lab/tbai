#include "tbai_deploy_spot/SpotRobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

void SpotRobotInterface::initMotorMapping() {
    // MuJoCo actuator order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    // MPC joint order: LF, LH, RF, RH (each: HAA, HFE, KFE)
    motorIdMap_["LF_HAA"] = 3;   // FL_hip
    motorIdMap_["LF_HFE"] = 4;   // FL_thigh
    motorIdMap_["LF_KFE"] = 5;   // FL_calf
    motorIdMap_["LH_HAA"] = 9;   // RL_hip
    motorIdMap_["LH_HFE"] = 10;  // RL_thigh
    motorIdMap_["LH_KFE"] = 11;  // RL_calf
    motorIdMap_["RF_HAA"] = 0;   // FR_hip
    motorIdMap_["RF_HFE"] = 1;   // FR_thigh
    motorIdMap_["RF_KFE"] = 2;   // FR_calf
    motorIdMap_["RH_HAA"] = 6;   // RR_hip
    motorIdMap_["RH_HFE"] = 7;   // RR_thigh
    motorIdMap_["RH_KFE"] = 8;   // RR_calf

    // Foot force sensor mapping (MuJoCo touch sensors: FR, FL, RR, RL)
    footIdMap_["LF_FOOT"] = 1;  // FL
    footIdMap_["RF_FOOT"] = 0;  // FR
    footIdMap_["LH_FOOT"] = 3;  // RL
    footIdMap_["RH_FOOT"] = 2;  // RR
}

SpotRobotInterface::SpotRobotInterface(SpotRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_spot");
    TBAI_LOG_INFO(logger_, "SpotRobotInterface constructor (tbai_sdk/zenoh backend)");

    initMotorMapping();

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", SPOT_TOPIC_LOWCMD);
    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>(SPOT_TOPIC_LOWCMD);

    // Initialize InEKF state estimator (foot names in LF, RF, LH, RH order)
    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, "");
    TBAI_LOG_INFO(logger_, "InEKF estimator initialized");

    rectifyOrientation_ = tbai::fromGlobalConfig<bool>("inekf_estimator/rectify_orientation", true);
    removeGyroscopeBias_ = tbai::fromGlobalConfig<bool>("inekf_estimator/remove_gyroscope_bias", true);

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", SPOT_TOPIC_LOWSTATE);
    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        SPOT_TOPIC_LOWSTATE,
        [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); },
        1);
}

SpotRobotInterface::~SpotRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying SpotRobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void SpotRobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    static auto last_time = currentTime;
    static int count = 0;
    count++;

    constexpr int N = 1000;
    if (count % N == 0) {
        scalar_t time_diff = currentTime - last_time;
        double rate = N / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "Low state callback rate: {} Hz (count: {})", rate, count);
        last_time = currentTime;
    }

    // Extract joint positions and velocities
    // MuJoCo sensor order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    // State order must match config joint_names: LF, LH, RF, RH
    vector_t jointAngles(SPOT_NUM_JOINTS);
    vector_t jointVelocities(SPOT_NUM_JOINTS);

    // LF (FL in MuJoCo: indices 3,4,5)
    jointAngles[0] = low_state.motor_states[3].q;
    jointAngles[1] = low_state.motor_states[4].q;
    jointAngles[2] = low_state.motor_states[5].q;
    // LH (RL in MuJoCo: indices 9,10,11)
    jointAngles[3] = low_state.motor_states[9].q;
    jointAngles[4] = low_state.motor_states[10].q;
    jointAngles[5] = low_state.motor_states[11].q;
    // RF (FR in MuJoCo: indices 0,1,2)
    jointAngles[6] = low_state.motor_states[0].q;
    jointAngles[7] = low_state.motor_states[1].q;
    jointAngles[8] = low_state.motor_states[2].q;
    // RH (RR in MuJoCo: indices 6,7,8)
    jointAngles[9] = low_state.motor_states[6].q;
    jointAngles[10] = low_state.motor_states[7].q;
    jointAngles[11] = low_state.motor_states[8].q;

    // Same reordering for velocities
    jointVelocities[0] = low_state.motor_states[3].dq;
    jointVelocities[1] = low_state.motor_states[4].dq;
    jointVelocities[2] = low_state.motor_states[5].dq;
    jointVelocities[3] = low_state.motor_states[9].dq;
    jointVelocities[4] = low_state.motor_states[10].dq;
    jointVelocities[5] = low_state.motor_states[11].dq;
    jointVelocities[6] = low_state.motor_states[0].dq;
    jointVelocities[7] = low_state.motor_states[1].dq;
    jointVelocities[8] = low_state.motor_states[2].dq;
    jointVelocities[9] = low_state.motor_states[6].dq;
    jointVelocities[10] = low_state.motor_states[7].dq;
    jointVelocities[11] = low_state.motor_states[8].dq;

    // IMU data
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

    // Contact detection
    std::vector<bool> contactFlags(4, false);
    const double contact_threshold = 19.0;
    if (low_state.foot_force.size() >= 4) {
        contactFlags[0] = static_cast<double>(low_state.foot_force[footIdMap_["LF_FOOT"]]) >= contact_threshold;
        contactFlags[1] = static_cast<double>(low_state.foot_force[footIdMap_["RF_FOOT"]]) >= contact_threshold;
        contactFlags[2] = static_cast<double>(low_state.foot_force[footIdMap_["LH_FOOT"]]) >= contact_threshold;
        contactFlags[3] = static_cast<double>(low_state.foot_force[footIdMap_["RH_FOOT"]]) >= contact_threshold;
    }

    // Calculate dt
    static scalar_t lastTime = currentTime;
    scalar_t dt = currentTime - lastTime;
    lastTime = currentTime;

    // Pass joints in config order — matches Gazebo InekfRosStateSubscriber behavior

    // Update state estimator
    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags, rectifyOrientation_, enable_);

    State state;
    state.x = vector_t::Zero(SPOT_STATE_DIM);

    // Base orientation
    const quaternion_t baseQuaternion =
        rectifyOrientation_ ? quaternion_t(baseOrientation) : quaternion_t(estimator_->getBaseOrientation());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;
    state.x.segment<3>(3) = estimator_->getBasePosition();

    if (removeGyroscopeBias_) {
        state.x.segment<3>(6) = baseAngVel - estimator_->getGyroscopeBias();
    } else {
        state.x.segment<3>(6) = baseAngVel;
    }

    state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();
    state.x.segment<12>(12) = jointAngles;
    state.x.segment<12>(24) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    initialized_ = true;
}

void SpotRobotInterface::publish(std::vector<MotorCommand> commands) {
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
    motor_commands.commands.resize(SPOT_NUM_JOINTS);

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

void SpotRobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the Spot robot to initialize...");
    }
    TBAI_LOG_INFO(logger_, "Spot robot initialized");
}

State SpotRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
