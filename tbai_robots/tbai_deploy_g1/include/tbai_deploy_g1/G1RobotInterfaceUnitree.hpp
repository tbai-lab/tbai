#pragma once

#include <math.h>
#include <stdint.h>

#include <string>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>

#define SCHED_DEADLINE 6

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define G1_UNITREE_TOPIC_LOWCMD "rt/lowcmd"
#define G1_UNITREE_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

constexpr int G1_NUM_JOINTS = 29;
constexpr int G1_STATE_DIM = 3 + 3 + 3 + 3 + G1_NUM_JOINTS + G1_NUM_JOINTS;

struct G1RobotInterfaceUnitreeArgs {
    std::string networkInterface = "eth0";
    int unitreeChannel = 0;
    bool channelInit = true;
};

class G1RobotInterfaceUnitree : public RobotInterface {
   public:
    G1RobotInterfaceUnitree(G1RobotInterfaceUnitreeArgs args);
    virtual ~G1RobotInterfaceUnitree();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

    inline vector4_t getBaseQuaternion() {
        std::lock_guard<std::mutex> lock(latestStateMutex_);
        return baseQuaternion_;
    }

   private:
    void lowStateCallback(const void *message);
    void initMotorMapping();

    unitree_hg::msg::dds_::LowCmd_ low_cmd{};

    /* Publishers */
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher;

    /* Subscribers */
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber;

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

    bool enable_ = false;
    void enableEstimator() override { enable_ = true; }
    void disableEstimator() override { enable_ = false; }
};

}  // namespace tbai
