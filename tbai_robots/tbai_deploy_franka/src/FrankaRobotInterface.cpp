#include "tbai_deploy_franka/FrankaRobotInterface.hpp"

#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

void FrankaRobotInterface::initMotorMapping() {
    // Franka Panda 7-DOF arm + 2-DOF gripper
    // Map panda_joint* names (used by URDF/MPC) to MuJoCo (or real robot) motor indices (TODO: check real robot
    // indices)
    motorIdMap_["panda_joint1"] = 0;
    motorIdMap_["panda_joint2"] = 1;
    motorIdMap_["panda_joint3"] = 2;
    motorIdMap_["panda_joint4"] = 3;
    motorIdMap_["panda_joint5"] = 4;
    motorIdMap_["panda_joint6"] = 5;
    motorIdMap_["panda_joint7"] = 6;

    // Finger joints
    motorIdMap_["panda_finger_joint1"] = 7;
    motorIdMap_["panda_finger_joint2"] = 8;

    // Finger command joint names
    fingerCommandLeft_.joint_name = "panda_finger_joint1";
    fingerCommandRight_.joint_name = "panda_finger_joint2";
}

FrankaRobotInterface::FrankaRobotInterface(FrankaRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_franka");
    TBAI_LOG_INFO(logger_, "FrankaRobotInterface constructor (tbai_sdk/zenoh backend)");

    initMotorMapping();

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", FRANKA_TOPIC_LOWCMD);
    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>(FRANKA_TOPIC_LOWCMD);

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", FRANKA_TOPIC_LOWSTATE);
    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        FRANKA_TOPIC_LOWSTATE, [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); }, 1);

    TBAI_LOG_INFO(logger_, "Closing gripper on startup: {}", args.closeGripper);
    if (args.closeGripper) {
        closeGripper();
    } else {
        TBAI_LOG_WARN(logger_, "Gripper will be 'floppy' and will not be closed on startup");
    }
}

FrankaRobotInterface::~FrankaRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying FrankaRobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void FrankaRobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
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

    // Extract arm joint positions/velocities (7 DOF) and finger joint positions/velocities (2 DOF)
    const int numReported = static_cast<int>(low_state.motor_states.size());

    // Fixed-base state layout:
    //   [0, 7)    arm positions
    //   [7, 14)   arm velocities
    //   [14, 16)  finger positions
    //   [16, 18)  finger velocities
    State state;
    state.x = vector_t::Zero(FRANKA_STATE_DIM);

    // Arm state
    const int numArm = std::min(numReported, FRANKA_NUM_ARM_JOINTS);
    for (int i = 0; i < numArm; ++i) {
        state.x[i] = low_state.motor_states[i].q;
        state.x[FRANKA_NUM_ARM_JOINTS + i] = low_state.motor_states[i].dq;
    }

    // Finger state
    const int numFingers = std::min(numReported - FRANKA_NUM_ARM_JOINTS, FRANKA_NUM_FINGER_JOINTS);
    for (int i = 0; i < numFingers; ++i) {
        state.x[2 * FRANKA_NUM_ARM_JOINTS + i] = low_state.motor_states[FRANKA_NUM_ARM_JOINTS + i].q;
        state.x[2 * FRANKA_NUM_ARM_JOINTS + FRANKA_NUM_FINGER_JOINTS + i] =
            low_state.motor_states[FRANKA_NUM_ARM_JOINTS + i].dq;
    }

    state.timestamp = currentTime;
    state.contactFlags = {};

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    initialized_ = true;
}

void FrankaRobotInterface::publish(std::vector<MotorCommand> commands) {
    static auto last_publish_time = std::chrono::high_resolution_clock::now();
    static int publish_count = 0;
    publish_count++;

    constexpr int PUBLISH_N = 100;
    if (publish_count % PUBLISH_N == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_publish_time).count();
        double rate = (PUBLISH_N * 1000.0) / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 20.0, "Publish frequency: {} Hz (count: {})", rate, publish_count);
        last_publish_time = current_time;
    }

    robot_msgs::MotorCommands motor_commands;
    motor_commands.commands.resize(FRANKA_NUM_MUJOCO_MOTORS);

    bool leftProvided = false;
    bool rightProvided = false;
    for (const auto &command : commands) {
        const int motorId = motorIdMap_[command.joint_name];
        motor_commands.commands[motorId].q = command.desired_position;
        motor_commands.commands[motorId].dq = command.desired_velocity;
        motor_commands.commands[motorId].kp = command.kp;
        motor_commands.commands[motorId].kd = command.kd;
        motor_commands.commands[motorId].tau = command.torque_ff;

        if (motorId == motorIdMap_["panda_finger_joint1"]) leftProvided = true;
        if (motorId == motorIdMap_["panda_finger_joint2"]) rightProvided = true;
    }

    // Overlay internal finger commands for slots the caller didn't populate.
    {
        std::lock_guard<std::mutex> lock(fingerCommandMutex_);
        auto fill = [&](const MotorCommand &src) {
            const int motorId = motorIdMap_[src.joint_name];
            motor_commands.commands[motorId].q = src.desired_position;
            motor_commands.commands[motorId].dq = src.desired_velocity;
            motor_commands.commands[motorId].kp = src.kp;
            motor_commands.commands[motorId].kd = src.kd;
            motor_commands.commands[motorId].tau = src.torque_ff;
        };
        if (!leftProvided) fill(fingerCommandLeft_);
        if (!rightProvided) fill(fingerCommandRight_);
    }

    lowcmd_publisher->publish(motor_commands);
}

void FrankaRobotInterface::setFingerCommand(scalar_t desired_position, scalar_t desired_velocity, scalar_t kp,
                                            scalar_t kd, scalar_t torque_ff) {
    std::lock_guard<std::mutex> lock(fingerCommandMutex_);
    for (MotorCommand *cmd : {&fingerCommandLeft_, &fingerCommandRight_}) {
        cmd->desired_position = desired_position;
        cmd->desired_velocity = desired_velocity;
        cmd->kp = kp;
        cmd->kd = kd;
        cmd->torque_ff = torque_ff;
    }
}

void FrankaRobotInterface::openGripper(scalar_t kp, scalar_t kd) {
    setFingerCommand(FRANKA_GRIPPER_OPEN_POS, 0.0, kp, kd);
}

void FrankaRobotInterface::closeGripper(scalar_t kp, scalar_t kd) {
    setFingerCommand(FRANKA_GRIPPER_CLOSED_POS, 0.0, kp, kd);
}

void FrankaRobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the Franka robot to initialize...");
    }
    TBAI_LOG_INFO(logger_, "Franka robot initialized");
}

State FrankaRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

}  // namespace tbai
