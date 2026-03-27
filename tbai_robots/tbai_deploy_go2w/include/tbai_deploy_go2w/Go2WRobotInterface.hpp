#pragma once

#include <math.h>
#include <stdint.h>

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_deploy_go2w/Go2WConstants.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>

#define GO2W_TOPIC_LOWCMD "rt/lowcmd"
#define GO2W_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

struct Go2WRobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "eth0");
    TBAI_ARG_DEFAULT(int, unitreeChannel, 0);
    TBAI_ARG_DEFAULT(bool, channelInit, true);
    TBAI_ARG_DEFAULT(bool, useGroundTruthState, false);
};

class Go2WRobotInterface : public RobotInterface {
   public:
    Go2WRobotInterface(Go2WRobotInterfaceArgs args);
    virtual ~Go2WRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void initMotorMapping();

    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;

    std::unordered_map<std::string, int> motorIdMap_;

    bool initialized_ = false;

    scalar_t lastYaw_ = 0.0;
    std::mutex latestStateMutex_;
    State state_;

    std::shared_ptr<spdlog::logger> logger_;

    bool useGroundTruthState_ = false;

    bool enable_ = false;
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
