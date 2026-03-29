#pragma once

#include <math.h>
#include <stdint.h>

#include <string>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>

namespace tbai {

// 12 leg joints + 6 arm joints = 18 DOF (gripper excluded from control)
constexpr int SPOT_ARM_NUM_LEG_JOINTS = 12;
constexpr int SPOT_ARM_NUM_ARM_JOINTS = 6;
constexpr int SPOT_ARM_NUM_JOINTS = SPOT_ARM_NUM_LEG_JOINTS + SPOT_ARM_NUM_ARM_JOINTS;
constexpr int SPOT_ARM_NUM_MUJOCO_MOTORS = 19;  // 12 legs + 6 arm + 1 gripper
constexpr int SPOT_ARM_STATE_DIM = 3 + 3 + 3 + 3 + SPOT_ARM_NUM_JOINTS + SPOT_ARM_NUM_JOINTS;

struct SpotArmRobotInterfaceArgs {
    std::string networkInterface = "lo";
};

class SpotArmRobotInterface : public RobotInterface {
   public:
    SpotArmRobotInterface(SpotArmRobotInterfaceArgs args);
    virtual ~SpotArmRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const robot_msgs::LowState &message);

    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;

    std::unordered_map<std::string, int> motorIdMap_;
    std::unordered_map<std::string, int> footIdMap_;
    bool initialized_ = false;

    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    scalar_t lastYaw_ = 0.0;
    std::mutex latestStateMutex_;
    State state_;

    std::shared_ptr<spdlog::logger> logger_;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    bool enable_ = false;
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
