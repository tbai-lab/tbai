#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>

#include <math.h>
#include <stdint.h>

#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#define FRANKA_TOPIC_LOWCMD "rt/lowcmd"
#define FRANKA_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

// Franka Panda: 7 arm joints (+ 2 finger joints in MuJoCo, but not in MPC/URDF)
constexpr int FRANKA_NUM_ARM_JOINTS = 7;
constexpr int FRANKA_NUM_MUJOCO_MOTORS = 8;  // 7 arm + 1 finger in MuJoCo

// Fixed-base state: [7 joint_pos, 7 joint_vel] = 14
// (no base orientation/position/velocity for fixed-base manipulator)
constexpr int FRANKA_STATE_DIM = FRANKA_NUM_ARM_JOINTS + FRANKA_NUM_ARM_JOINTS;

struct FrankaRobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "lo");
    TBAI_ARG_DEFAULT(bool, enableCamera, false);
    TBAI_ARG_DEFAULT(std::string, cameraTopic, "rt/camera/image");
};

class FrankaRobotInterface : public RobotInterface {
   public:
    FrankaRobotInterface(FrankaRobotInterfaceArgs args);
    virtual ~FrankaRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

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

    std::shared_ptr<spdlog::logger> logger_;

    bool enable_ = false;
    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }
};

}  // namespace tbai
