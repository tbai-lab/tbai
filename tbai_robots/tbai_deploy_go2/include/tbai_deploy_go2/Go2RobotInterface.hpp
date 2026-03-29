#pragma once

#include <math.h>
#include <stdint.h>

#include <atomic>

#include <string>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

namespace tbai {

struct Go2RobotInterfaceArgs {
    std::string networkInterface = "eth0";
    int unitreeChannel = 0;
    bool channelInit = true;
    bool enableStateEstim = true;
    bool subscribeLidar = true;
    bool enableVideo = false;
    bool useGroundTruthState = false;
};

class Go2RobotInterface : public RobotInterface {
   public:
    Go2RobotInterface(Go2RobotInterfaceArgs args);
    virtual ~Go2RobotInterface();

    // virtual methods from RobotInterface
    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const robot_msgs::LowState &message);

    /* Publishers */
    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;

    /* Subscribers */
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

    bool useGroundTruthState_ = false;

    bool enable_ = false;  // Enable state estimation
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
