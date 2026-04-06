#include "tbai_deploy_go2w/Go2WRobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

Go2WRobotInterface::Go2WRobotInterface(Go2WRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_go2w");
    TBAI_LOG_INFO(logger_, "Go2WRobotInterface constructor (tbai_sdk/zenoh backend)");

    // Motor mapping: MuJoCo order matches real robot order
    // FR(0-2), FL(3-5), RR(6-8), RL(9-11), wheels(12-15)
    motorIdMap_["FR_hip_joint"] = 0;
    motorIdMap_["FR_thigh_joint"] = 1;
    motorIdMap_["FR_calf_joint"] = 2;
    motorIdMap_["FL_hip_joint"] = 3;
    motorIdMap_["FL_thigh_joint"] = 4;
    motorIdMap_["FL_calf_joint"] = 5;
    motorIdMap_["RR_hip_joint"] = 6;
    motorIdMap_["RR_thigh_joint"] = 7;
    motorIdMap_["RR_calf_joint"] = 8;
    motorIdMap_["RL_hip_joint"] = 9;
    motorIdMap_["RL_thigh_joint"] = 10;
    motorIdMap_["RL_calf_joint"] = 11;
    motorIdMap_["FR_foot_joint"] = 12;
    motorIdMap_["FL_foot_joint"] = 13;
    motorIdMap_["RR_foot_joint"] = 14;
    motorIdMap_["RL_foot_joint"] = 15;

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", GO2W_TOPIC_LOWCMD);
    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>(GO2W_TOPIC_LOWCMD);

    useGroundTruthState_ = args.useGroundTruthState;
    if (useGroundTruthState_) {
        TBAI_LOG_INFO(logger_, "Using ground-truth position/velocity from LowState (when available)");
    }

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", GO2W_TOPIC_LOWSTATE);
    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        GO2W_TOPIC_LOWSTATE, [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); }, 1);
}

Go2WRobotInterface::~Go2WRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying Go2WRobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void Go2WRobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    static auto last_time = currentTime;
    static int count = 0;
    count++;

    constexpr int N = 200;
    if (count % N == 0) {
        scalar_t time_diff = currentTime - last_time;
        double rate = N / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "Low state callback rate: {} Hz (count: {})", rate, count);
        last_time = currentTime;
    }

    vector_t jointAngles(go2w::GO2W_NUM_JOINTS);
    vector_t jointVelocities(go2w::GO2W_NUM_JOINTS);

    int numMotors = std::min(static_cast<int>(low_state.motor_states.size()), go2w::GO2W_NUM_JOINTS);
    for (int i = 0; i < numMotors; ++i) {
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

    std::vector<bool> contactFlags(4, true);

    State state;
    state.x = vector_t::Zero(go2w::GO2W_STATE_DIM);

    const quaternion_t baseQuaternion(baseOrientation);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;
    if (useGroundTruthState_ && low_state.has_position && low_state.has_velocity) {
        const tbai::matrix3_t R_base_world = R_world_base.transpose();
        vector3_t gtPos(low_state.position[0], low_state.position[1], low_state.position[2]);
        vector3_t gtVel(low_state.velocity[0], low_state.velocity[1], low_state.velocity[2]);
        state.x.segment<3>(3) = gtPos;
        state.x.segment<3>(9) = R_base_world * gtVel;
    } else {
        state.x.segment<3>(3).setZero();
        state.x.segment<3>(9).setZero();
    }
    state.x.segment<3>(6) = baseAngVel;
    state.x.segment(12, go2w::GO2W_NUM_JOINTS) = jointAngles;
    state.x.segment(12 + go2w::GO2W_NUM_JOINTS, go2w::GO2W_NUM_JOINTS) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    initialized_ = true;
}

void Go2WRobotInterface::publish(std::vector<MotorCommand> commands) {
    static auto last_publish_time = std::chrono::high_resolution_clock::now();
    static int publish_count = 0;
    publish_count++;

    constexpr int PUBLISH_N = 100;
    if (publish_count % PUBLISH_N == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_publish_time).count();
        double rate = (PUBLISH_N * 1000.0) / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 30.0, "Publish frequency: {} Hz (count: {})", rate, publish_count);
        last_publish_time = current_time;
    }

    robot_msgs::MotorCommands motor_commands;
    motor_commands.commands.resize(go2w::GO2W_NUM_JOINTS);

    for (const auto &command : commands) {
        int motor_id = motorIdMap_[command.joint_name];
        motor_commands.commands[motor_id].q = command.desired_position;
        motor_commands.commands[motor_id].dq = command.desired_velocity;
        motor_commands.commands[motor_id].kp = command.kp;
        motor_commands.commands[motor_id].kd = command.kd;
        motor_commands.commands[motor_id].tau = command.torque_ff;
    }

    lowcmd_publisher->publish(motor_commands);
}

void Go2WRobotInterface::waitTillInitialized() {
    TBAI_LOG_INFO(logger_, "Waiting for the robot to initialize...");
    while (!initialized_) {
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the robot to initialize...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    TBAI_LOG_INFO(logger_, "Robot initialized");
}

State Go2WRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
