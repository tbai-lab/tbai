#include "tbai_deploy_g1/G1RobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
static uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }

            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

namespace tbai {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void G1RobotInterface::initMotorMapping() {
    // G1 29DOF motor mapping according to unitree documentation
    // Left leg (6 DOF)
    motorIdMap_["left_hip_pitch_joint"] = 0;
    motorIdMap_["left_hip_roll_joint"] = 1;
    motorIdMap_["left_hip_yaw_joint"] = 2;
    motorIdMap_["left_knee_joint"] = 3;
    motorIdMap_["left_ankle_pitch_joint"] = 4;
    motorIdMap_["left_ankle_roll_joint"] = 5;

    // Right leg (6 DOF)
    motorIdMap_["right_hip_pitch_joint"] = 6;
    motorIdMap_["right_hip_roll_joint"] = 7;
    motorIdMap_["right_hip_yaw_joint"] = 8;
    motorIdMap_["right_knee_joint"] = 9;
    motorIdMap_["right_ankle_pitch_joint"] = 10;
    motorIdMap_["right_ankle_roll_joint"] = 11;

    // Waist (3 DOF)
    motorIdMap_["waist_yaw_joint"] = 12;
    motorIdMap_["waist_roll_joint"] = 13;
    motorIdMap_["waist_pitch_joint"] = 14;

    // Left arm (7 DOF)
    motorIdMap_["left_shoulder_pitch_joint"] = 15;
    motorIdMap_["left_shoulder_roll_joint"] = 16;
    motorIdMap_["left_shoulder_yaw_joint"] = 17;
    motorIdMap_["left_elbow_joint"] = 18;
    motorIdMap_["left_wrist_roll_joint"] = 19;
    motorIdMap_["left_wrist_pitch_joint"] = 20;
    motorIdMap_["left_wrist_yaw_joint"] = 21;

    // Right arm (7 DOF)
    motorIdMap_["right_shoulder_pitch_joint"] = 22;
    motorIdMap_["right_shoulder_roll_joint"] = 23;
    motorIdMap_["right_shoulder_yaw_joint"] = 24;
    motorIdMap_["right_elbow_joint"] = 25;
    motorIdMap_["right_wrist_roll_joint"] = 26;
    motorIdMap_["right_wrist_pitch_joint"] = 27;
    motorIdMap_["right_wrist_yaw_joint"] = 28;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
G1RobotInterface::G1RobotInterface(G1RobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_g1");
    TBAI_LOG_INFO(logger_, "G1RobotInterface constructor");
    TBAI_LOG_INFO(logger_, "Network interface: {}", args.networkInterface());
    TBAI_LOG_INFO(logger_, "Initializing G1RobotInterface (29 DOF)");
    TBAI_LOG_INFO(logger_, "Unitree channel: {}", args.unitreeChannel());
    TBAI_LOG_INFO(logger_, "Channel init: {}", args.channelInit());

    if (args.channelInit()) {
        TBAI_LOG_INFO(logger_, "Initializing channel factory: {}", args.networkInterface());
        unitree::robot::ChannelFactory::Instance()->Init(args.unitreeChannel(), args.networkInterface());
    } else {
        throw std::runtime_error("Channel init is disabled");
    }

    // Initialize motor ID mapping
    initMotorMapping();

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", G1_TOPIC_LOWCMD);
    lowcmd_publisher.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(G1_TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    // Set mode_machine to 5 for 29DOF G1
    low_cmd.mode_machine() = 5;

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", G1_TOPIC_LOWSTATE);
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(G1_TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&G1RobotInterface::lowStateCallback, this, std::placeholders::_1), 1);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
G1RobotInterface::~G1RobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying G1RobotInterface");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void G1RobotInterface::lowStateCallback(const void *message) {
    auto t11 = std::chrono::high_resolution_clock::now();
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    // Create a copy of the low level state
    unitree_hg::msg::dds_::LowState_ &low_state = *(unitree_hg::msg::dds_::LowState_ *)message;

    // Calculate callback rate
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

    // Extract joint positions and velocities from low_state (29 DOF)
    vector_t jointAngles(G1_NUM_JOINTS);
    vector_t jointVelocities(G1_NUM_JOINTS);

    // Map all 29 joints
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        jointAngles[i] = low_state.motor_state()[i].q();
        jointVelocities[i] = low_state.motor_state()[i].dq();
    }

    // Extract IMU data
    // Unitree IMU provides quaternion in [w, x, y, z] order
    // We store as [x, y, z, w] for Eigen compatibility and TWIST2
    vector4_t baseOrientation;
    baseOrientation[0] = low_state.imu_state().quaternion()[1];  // x
    baseOrientation[1] = low_state.imu_state().quaternion()[2];  // y
    baseOrientation[2] = low_state.imu_state().quaternion()[3];  // z
    baseOrientation[3] = low_state.imu_state().quaternion()[0];  // w

    vector3_t baseAngVel;
    baseAngVel[0] = low_state.imu_state().gyroscope()[0];
    baseAngVel[1] = low_state.imu_state().gyroscope()[1];
    baseAngVel[2] = low_state.imu_state().gyroscope()[2];

    // Get the state
    State state;
    state.x = vector_t::Zero(G1_STATE_DIM);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    const quaternion_t baseQuaternion = quaternion_t(baseOrientation);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    state.x.segment<3>(0) = rpy;

    // Base position (estimated from IMU, no absolute position without external tracking)
    // TODO(lnotspotl): Add state estimator
    state.x.segment<3>(3).setZero();

    // Base angular velocity
    state.x.segment<3>(6) = baseAngVel;

    // Base linear velocity (estimated, zero without proper state estimation)
    // TODO(lnotspotl): Add state estimator
    state.x.segment<3>(9).setZero();

    // Joint positions (29 DOF)
    state.x.segment(12, G1_NUM_JOINTS) = jointAngles;

    // Joint velocities (29 DOF)
    state.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS) = jointVelocities;

    state.timestamp = currentTime;

    // G1 bipedal - 2 contact points (feet)
    // TODO(lnotspotl): Add state estimator
    state.contactFlags = {true, true};

    auto t12 = std::chrono::high_resolution_clock::now();
    TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "State update time: {} us",
                           std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count());

    // Update the latest state
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    baseQuaternion_ = std::move(baseOrientation);

    initialized_ = true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
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

    for (const auto &command : commands) {
        const int motorId = motorIdMap_[command.joint_name];
        low_cmd.motor_cmd()[motorId].mode() = 1;  // Position mode
        low_cmd.motor_cmd()[motorId].q() = command.desired_position;
        low_cmd.motor_cmd()[motorId].kp() = command.kp;
        low_cmd.motor_cmd()[motorId].dq() = command.desired_velocity;
        low_cmd.motor_cmd()[motorId].kd() = command.kd;
        low_cmd.motor_cmd()[motorId].tau() = command.torque_ff;
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);

    // Publish the low level command
    lowcmd_publisher->Write(low_cmd);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void G1RobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        TBAI_LOG_INFO_THROTTLE(logger_, 3.0, "Waiting for the G1 robot to initialize...");
    }
    TBAI_LOG_DEBUG(logger_, "G1 Robot initialized");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
State G1RobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
