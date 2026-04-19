#pragma once

#include <math.h>
#include <stdint.h>

#include <string>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>

#define FRANKA_TOPIC_LOWCMD "rt/lowcmd"
#define FRANKA_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

// Franka Panda: 7 arm joints + 2 finger joints
constexpr int FRANKA_NUM_ARM_JOINTS = 7;
constexpr int FRANKA_NUM_FINGER_JOINTS = 2;
constexpr int FRANKA_NUM_MUJOCO_MOTORS = FRANKA_NUM_ARM_JOINTS + FRANKA_NUM_FINGER_JOINTS;

// Fixed-base state: [7 arm_pos, 7 arm_vel, 2 finger_pos, 2 finger_vel] = 18
// Arm segments are kept first so arm MPC observation (head(7), segment(7,7)) still works.
constexpr int FRANKA_STATE_DIM = 2 * FRANKA_NUM_ARM_JOINTS + 2 * FRANKA_NUM_FINGER_JOINTS;

// Default gripper travel limits and PD gains (per-finger prismatic joint, meters).
constexpr scalar_t FRANKA_GRIPPER_OPEN_POS = 0.04;
constexpr scalar_t FRANKA_GRIPPER_CLOSED_POS = 0.0;
constexpr scalar_t FRANKA_GRIPPER_DEFAULT_KP = 200.0;
constexpr scalar_t FRANKA_GRIPPER_DEFAULT_KD = 10.0;


struct FrankaRobotInterfaceArgs {
    bool closeGripper = true;
};

class FrankaRobotInterface : public RobotInterface {
   public:
    FrankaRobotInterface(FrankaRobotInterfaceArgs args);
    virtual ~FrankaRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

    /**
     * Set the commanded state of both gripper fingers (shared Kp/Kd/q/dq).
     * The command persists across publish() calls until updated again.
     * publish() overlays these on top of whatever arm commands the controller sent.
     */
    void setFingerCommand(scalar_t desired_position, scalar_t desired_velocity, scalar_t kp, scalar_t kd,
                          scalar_t torque_ff = 0.0);

    /** Command the gripper to fully open (0.04 m per finger) with default PD gains. */
    void openGripper(scalar_t kp = FRANKA_GRIPPER_DEFAULT_KP, scalar_t kd = FRANKA_GRIPPER_DEFAULT_KD);

    /** Command the gripper to fully close (0.0 m per finger) with default PD gains. */
    void closeGripper(scalar_t kp = FRANKA_GRIPPER_DEFAULT_KP, scalar_t kd = FRANKA_GRIPPER_DEFAULT_KD);

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void initMotorMapping();

    /* Publishers */
    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;

    /* Subscribers */
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;

    std::unordered_map<std::string, int> motorIdMap_;
    bool initialized_ = false;

    std::mutex latestStateMutex_;
    State state_;

    std::mutex fingerCommandMutex_;
    MotorCommand fingerCommandLeft_{};
    MotorCommand fingerCommandRight_{};

    std::shared_ptr<spdlog::logger> logger_;

    bool enable_ = false;
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
