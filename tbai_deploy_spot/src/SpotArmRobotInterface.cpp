#include "tbai_deploy_spot/SpotArmRobotInterface.hpp"

#include <stdint.h>
#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

SpotArmRobotInterface::SpotArmRobotInterface(SpotArmRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_spot_arm");
    TBAI_LOG_INFO(logger_, "SpotArmRobotInterface constructor (tbai_sdk/zenoh backend)");

    // MuJoCo actuator order: FR(0-2), FL(3-5), RR(6-8), RL(9-11), arm(12-17), gripper(18)
    // Config joint_names order: LF, LH, RF, RH, arm
    motorIdMap_["LF_HAA"] = 3;   motorIdMap_["LF_HFE"] = 4;   motorIdMap_["LF_KFE"] = 5;
    motorIdMap_["LH_HAA"] = 9;   motorIdMap_["LH_HFE"] = 10;  motorIdMap_["LH_KFE"] = 11;
    motorIdMap_["RF_HAA"] = 0;   motorIdMap_["RF_HFE"] = 1;   motorIdMap_["RF_KFE"] = 2;
    motorIdMap_["RH_HAA"] = 6;   motorIdMap_["RH_HFE"] = 7;   motorIdMap_["RH_KFE"] = 8;
    motorIdMap_["arm_sh0"] = 12; motorIdMap_["arm_sh1"] = 13;
    motorIdMap_["arm_el0"] = 14; motorIdMap_["arm_el1"] = 15;
    motorIdMap_["arm_wr0"] = 16; motorIdMap_["arm_wr1"] = 17;

    // Bridge foot_force order: FR(0), FL(1), RR(2), RL(3)
    footIdMap_["LF_FOOT"] = 1;  footIdMap_["RF_FOOT"] = 0;
    footIdMap_["LH_FOOT"] = 3;  footIdMap_["RH_FOOT"] = 2;

    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>("rt/lowcmd");

    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, "");

    rectifyOrientation_ = tbai::fromGlobalConfig<bool>("inekf_estimator/rectify_orientation", true);
    removeGyroscopeBias_ = tbai::fromGlobalConfig<bool>("inekf_estimator/remove_gyroscope_bias", true);

    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        "rt/lowstate",
        [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); },
        1);
}

SpotArmRobotInterface::~SpotArmRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying SpotArmRobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void SpotArmRobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
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

    // MuJoCo sensor order: FR(0-2), FL(3-5), RR(6-8), RL(9-11), arm(12-17), gripper(18)
    // State order must match config: LF, LH, RF, RH, arm
    vector_t jointAngles(SPOT_ARM_NUM_JOINTS);
    vector_t jointVelocities(SPOT_ARM_NUM_JOINTS);

    // LF (FL in MuJoCo: 3-5)
    jointAngles[0] = low_state.motor_states[3].q;   jointVelocities[0] = low_state.motor_states[3].dq;
    jointAngles[1] = low_state.motor_states[4].q;   jointVelocities[1] = low_state.motor_states[4].dq;
    jointAngles[2] = low_state.motor_states[5].q;   jointVelocities[2] = low_state.motor_states[5].dq;
    // LH (RL in MuJoCo: 9-11)
    jointAngles[3] = low_state.motor_states[9].q;   jointVelocities[3] = low_state.motor_states[9].dq;
    jointAngles[4] = low_state.motor_states[10].q;  jointVelocities[4] = low_state.motor_states[10].dq;
    jointAngles[5] = low_state.motor_states[11].q;  jointVelocities[5] = low_state.motor_states[11].dq;
    // RF (FR in MuJoCo: 0-2)
    jointAngles[6] = low_state.motor_states[0].q;   jointVelocities[6] = low_state.motor_states[0].dq;
    jointAngles[7] = low_state.motor_states[1].q;   jointVelocities[7] = low_state.motor_states[1].dq;
    jointAngles[8] = low_state.motor_states[2].q;   jointVelocities[8] = low_state.motor_states[2].dq;
    // RH (RR in MuJoCo: 6-8)
    jointAngles[9] = low_state.motor_states[6].q;   jointVelocities[9] = low_state.motor_states[6].dq;
    jointAngles[10] = low_state.motor_states[7].q;  jointVelocities[10] = low_state.motor_states[7].dq;
    jointAngles[11] = low_state.motor_states[8].q;  jointVelocities[11] = low_state.motor_states[8].dq;
    // Arm (MuJoCo: 12-17)
    for (int i = 0; i < SPOT_ARM_NUM_ARM_JOINTS; ++i) {
        jointAngles[12 + i] = low_state.motor_states[12 + i].q;
        jointVelocities[12 + i] = low_state.motor_states[12 + i].dq;
    }

    vector4_t baseOrientation;
    baseOrientation[0] = low_state.imu_state.quaternion[1];
    baseOrientation[1] = low_state.imu_state.quaternion[2];
    baseOrientation[2] = low_state.imu_state.quaternion[3];
    baseOrientation[3] = low_state.imu_state.quaternion[0];

    vector3_t baseAngVel;
    baseAngVel[0] = low_state.imu_state.gyroscope[0];
    baseAngVel[1] = low_state.imu_state.gyroscope[1];
    baseAngVel[2] = low_state.imu_state.gyroscope[2];

    vector3_t baseAcc;
    baseAcc[0] = low_state.imu_state.accelerometer[0];
    baseAcc[1] = low_state.imu_state.accelerometer[1];
    baseAcc[2] = low_state.imu_state.accelerometer[2];

    std::vector<bool> contactFlags(4, false);
    const double contact_threshold = 19.0;
    if (low_state.foot_force.size() >= 4) {
        contactFlags[0] = static_cast<double>(low_state.foot_force[footIdMap_["LF_FOOT"]]) >= contact_threshold;
        contactFlags[1] = static_cast<double>(low_state.foot_force[footIdMap_["RF_FOOT"]]) >= contact_threshold;
        contactFlags[2] = static_cast<double>(low_state.foot_force[footIdMap_["LH_FOOT"]]) >= contact_threshold;
        contactFlags[3] = static_cast<double>(low_state.foot_force[footIdMap_["RH_FOOT"]]) >= contact_threshold;
    }

    static scalar_t lastTime = currentTime;
    scalar_t dt = currentTime - lastTime;
    lastTime = currentTime;

    // Pass joints in config order — matches Gazebo InekfRosStateSubscriber behavior
    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities,
                       baseAcc, baseAngVel, contactFlags, rectifyOrientation_, enable_);

    State state;
    state.x = vector_t::Zero(SPOT_ARM_STATE_DIM);

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
    state.x.segment(12, SPOT_ARM_NUM_JOINTS) = jointAngles;
    state.x.segment(12 + SPOT_ARM_NUM_JOINTS, SPOT_ARM_NUM_JOINTS) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    initialized_ = true;
}

void SpotArmRobotInterface::publish(std::vector<MotorCommand> commands) {
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
    motor_commands.commands.resize(SPOT_ARM_NUM_MUJOCO_MOTORS);

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

void SpotArmRobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the Spot Arm robot to initialize...");
    }
    TBAI_LOG_INFO(logger_, "Spot Arm robot initialized");
}

State SpotArmRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
