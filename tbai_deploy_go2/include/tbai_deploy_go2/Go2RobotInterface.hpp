#pragma once

#include <tbai_core/Args.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/RobotInterface.hpp>

#include <atomic>
#include <math.h>
#include <stdint.h>

#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LIDAR "rt/utlidar/cloud"
#define TOPIC_POINTCLOUD "rt/pointcloud"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

namespace tbai {

struct Go2RobotInterfaceArgs {
    TBAI_ARG_DEFAULT(std::string, networkInterface, "eth0");
    TBAI_ARG_DEFAULT(int, unitreeChannel, 0);
    TBAI_ARG_DEFAULT(bool, channelInit, true);
    TBAI_ARG_DEFAULT(bool, enableStateEstim, true);
    TBAI_ARG_DEFAULT(bool, subscribeLidar, true);
    TBAI_ARG_DEFAULT(bool, enableVideo, false);
    TBAI_ARG_DEFAULT(bool, subscribePointcloud, false);
    TBAI_ARG_DEFAULT(std::string, pointcloudTopic, "rt/pointcloud");
    TBAI_ARG_DEFAULT(bool, useGroundTruthState, false);
};

class Go2RobotInterface : public RobotInterface {
   public:
    Go2RobotInterface(Go2RobotInterfaceArgs args);
    virtual ~Go2RobotInterface();

    // virtual methods from RobotInterface
    void publish(std::vector<MotorCommand> commands) override;
    void waitTillInitialized() override;
    State getLatestState() override;

    virtual void lidarCallback(const robot_msgs::PointCloud2 &message) {
        // Do nothing by default, a user is expected to override this method, but does not have to
    };

    std::vector<float> getLatestPointcloud();

   private:
    void lowStateCallback(const robot_msgs::LowState &message);
    void pointcloudCallback(const robot_msgs::PointCloud2 &message);

    /* Publishers */
    std::unique_ptr<tbai::Publisher<robot_msgs::MotorCommands>> lowcmd_publisher;

    /* Subscribers */
    std::unique_ptr<tbai::QueuedSubscriber<robot_msgs::LowState>> lowstate_subscriber;
    std::unique_ptr<tbai::Subscriber<robot_msgs::PointCloud2>> lidar_subscriber;
    std::unique_ptr<tbai::Subscriber<robot_msgs::PointCloud2>> pointcloud_subscriber;

    std::unordered_map<std::string, int> motorIdMap_;
    std::unordered_map<std::string, int> footIdMap_;
    bool initialized_ = false;

    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    scalar_t lastYaw_ = 0.0;
    std::mutex latestStateMutex_;
    State state_;

    std::mutex pointcloudMutex_;
    std::vector<float> latestPointcloud_;  // Nx3 flattened (x,y,z,x,y,z,...)

    std::shared_ptr<spdlog::logger> logger_;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    bool useGroundTruthState_ = false;

    bool enable_ = false;  // Enable state estimation
    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }
};

}  // namespace tbai
