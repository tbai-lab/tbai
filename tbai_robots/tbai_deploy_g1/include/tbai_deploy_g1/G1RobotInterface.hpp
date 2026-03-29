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

#define G1_TOPIC_LOWCMD "rt/lowcmd"
#define G1_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

// G1 29DOF joint count
constexpr int G1_NUM_JOINTS = 29;

// State vector size: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 29 joint pos + 29 joint vel
constexpr int G1_STATE_DIM = 3 + 3 + 3 + 3 + G1_NUM_JOINTS + G1_NUM_JOINTS;

struct G1RobotInterfaceArgs {
    std::string networkInterface = "eth0";
    int unitreeChannel = 0;
    bool channelInit = true;
    bool enableStateEstim = true;
    bool useGroundTruthState = false;
};

class G1RobotInterface : public RobotInterface {
   public:
    G1RobotInterface(G1RobotInterfaceArgs args);
    virtual ~G1RobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

    inline vector4_t getBaseQuaternion() {
        std::lock_guard<std::mutex> lock(latestStateMutex_);
        return baseQuaternion_;
    }

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void initMotorMapping();

    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;

    std::unordered_map<std::string, int> motorIdMap_;
    bool initialized_ = false;

    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    scalar_t lastYaw_ = 0.0;
    std::mutex latestStateMutex_;

    State state_;
    vector4_t baseQuaternion_;

    std::shared_ptr<spdlog::logger> logger_;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    bool useGroundTruthState_ = false;

    bool enable_ = false;
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
