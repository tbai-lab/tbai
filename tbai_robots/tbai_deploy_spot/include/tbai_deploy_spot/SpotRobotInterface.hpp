#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>

#include <math.h>
#include <stdint.h>

#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#define SPOT_TOPIC_LOWCMD "rt/lowcmd"
#define SPOT_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

constexpr int SPOT_NUM_JOINTS = 12;

// State: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 12 joint pos + 12 joint vel = 36
constexpr int SPOT_STATE_DIM = 3 + 3 + 3 + 3 + SPOT_NUM_JOINTS + SPOT_NUM_JOINTS;

struct SpotRobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "lo");
};

class SpotRobotInterface : public RobotInterface {
   public:
    SpotRobotInterface(SpotRobotInterfaceArgs args);
    virtual ~SpotRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void initMotorMapping();

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
    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }
};

}  // namespace tbai
