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

    // Forward visual-odometry landmark observations (3D points in the body
    // frame, with persistent integer IDs) to the internal InEKFEstimator.
    // Safe to call from any thread; protected by an internal mutex.
    // When `pruneStale` is true, drops any tracked landmark not in this
    // observation set -- essential for real-time use, see the InEKFEstimator
    // declaration for why.
    void correctVisualLandmarks(const ::inekf::vectorLandmarks &landmarks, bool pruneStale = true);

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
    // Guards every call into estimator_->*, since the IMU-driven update path
    // (lowStateCallback) and the visual-landmark correction path
    // (correctVisualLandmarks) run on different threads.
    std::mutex estimatorMutex_;
    // Cached estimator readouts. Populated whenever the IMU callback
    // successfully takes estimatorMutex_, and reused on samples where the
    // try_lock fails (the VO worker is holding the mutex). This keeps the
    // published State sane even when the estimator update is skipped.
    vector3_t cachedBasePosition_ = vector3_t::Zero();
    vector3_t cachedBaseVelocity_ = vector3_t::Zero();
    vector3_t cachedGyroBias_ = vector3_t::Zero();
    quaternion_t cachedBaseQuaternion_ = quaternion_t::Identity();
    std::atomic<size_t> estimatorBusySkips_{0};

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
