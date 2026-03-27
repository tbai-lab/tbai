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

#define ANYMAL_C_TOPIC_LOWCMD "rt/lowcmd"
#define ANYMAL_C_TOPIC_LOWSTATE "rt/lowstate"

namespace tbai {

constexpr int ANYMAL_C_NUM_JOINTS = 12;
constexpr int ANYMAL_C_STATE_DIM = 3 + 3 + 3 + 3 + ANYMAL_C_NUM_JOINTS + ANYMAL_C_NUM_JOINTS;

struct AnymalCRobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "lo");
    TBAI_ARG_DEFAULT(bool, enableGroundPlaneCorrection, false);
    TBAI_ARG_DEFAULT(std::string, depthTopic, "rt/pointcloud/front_lower");
    TBAI_ARG_DEFAULT(bool, useGroundTruthState, false);
};

class AnymalCRobotInterface : public RobotInterface {
   public:
    AnymalCRobotInterface(AnymalCRobotInterfaceArgs args);
    virtual ~AnymalCRobotInterface();

    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void initMotorMapping();

    // Ground plane estimation from depth pointcloud
    void updateGroundPlaneEstimate();
    double estimateGroundHeight(const robot_msgs::PointCloud2 &pc);

    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;

    // Depth pointcloud subscriber for ground plane correction
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::PointCloud2>> depthSubscriber_;
    bool enableGroundPlaneCorrection_ = true;
    double groundHeightEstimate_ = 0.0;
    bool groundHeightValid_ = false;

    // Use ground-truth position/velocity from LowState (if available)
    bool useGroundTruthState_ = false;

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
