#include "tbai_deploy_anymal_c/AnymalCRobotInterface.hpp"

#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/Rate.hpp>

namespace tbai {

void AnymalCRobotInterface::initMotorMapping() {
    // MuJoCo actuator order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
    // Config joint_names order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
    motorIdMap_["LF_HAA"] = 0;
    motorIdMap_["LF_HFE"] = 1;
    motorIdMap_["LF_KFE"] = 2;
    motorIdMap_["LH_HAA"] = 6;
    motorIdMap_["LH_HFE"] = 7;
    motorIdMap_["LH_KFE"] = 8;
    motorIdMap_["RF_HAA"] = 3;
    motorIdMap_["RF_HFE"] = 4;
    motorIdMap_["RF_KFE"] = 5;
    motorIdMap_["RH_HAA"] = 9;
    motorIdMap_["RH_HFE"] = 10;
    motorIdMap_["RH_KFE"] = 11;

    // Bridge foot_force order: FR(0), FL(1), RR(2), RL(3)
    // LF=FL(1), RF=FR(0), LH=RL(3), RH=RR(2)
    footIdMap_["LF_FOOT"] = 1;
    footIdMap_["RF_FOOT"] = 0;
    footIdMap_["LH_FOOT"] = 3;
    footIdMap_["RH_FOOT"] = 2;
}

AnymalCRobotInterface::AnymalCRobotInterface(AnymalCRobotInterfaceArgs args) {
    logger_ = tbai::getLogger("tbai_deploy_anymal_c");
    TBAI_LOG_INFO(logger_, "AnymalCRobotInterface constructor (tbai_sdk/zenoh backend)");

    initMotorMapping();

    TBAI_LOG_INFO(logger_, "Initializing publisher: Topic: {}", ANYMAL_C_TOPIC_LOWCMD);
    lowcmd_publisher = std::make_unique<tbai::Publisher<robot_msgs::MotorCommands>>(ANYMAL_C_TOPIC_LOWCMD);

    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, "");
    TBAI_LOG_INFO(logger_, "InEKF estimator initialized");

    rectifyOrientation_ = tbai::fromGlobalConfig<bool>("inekf_estimator/rectify_orientation", true);
    removeGyroscopeBias_ = tbai::fromGlobalConfig<bool>("inekf_estimator/remove_gyroscope_bias", false);

    enableGroundPlaneCorrection_ = args.enableGroundPlaneCorrection;
    if (enableGroundPlaneCorrection_) {
        std::string depthTopic = args.depthTopic;
        TBAI_LOG_INFO(logger_, "Initializing depth subscriber for ground plane correction: {}", depthTopic);
        depthSubscriber_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::PointCloud2>>(depthTopic);
    }

    useGroundTruthState_ = args.useGroundTruthState;
    if (useGroundTruthState_) {
        TBAI_LOG_INFO(logger_, "Using ground-truth position/velocity from LowState (when available)");
    }

    TBAI_LOG_INFO(logger_, "Initializing subscriber - Topic: {}", ANYMAL_C_TOPIC_LOWSTATE);
    lowstate_subscriber = std::make_unique<tbai::QueuedSubscriber<robot_msgs::LowState>>(
        ANYMAL_C_TOPIC_LOWSTATE, [this](const robot_msgs::LowState &msg) { lowStateCallback(msg); }, 1);
}

AnymalCRobotInterface::~AnymalCRobotInterface() {
    TBAI_LOG_INFO(logger_, "Destroying AnymalCRobotInterface");
    if (lowstate_subscriber) lowstate_subscriber->stop();
}

void AnymalCRobotInterface::lowStateCallback(const robot_msgs::LowState &low_state) {
    scalar_t currentTime = tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow();

    static auto last_time = currentTime;
    static int count = 0;
    count++;

    constexpr int N = 1000;
    if (count % N == 0) {
        scalar_t time_diff = currentTime - last_time;
        double rate = N / time_diff;
        TBAI_LOG_INFO_THROTTLE(logger_, 8.0, "Low state callback rate: {} Hz (count: {})", rate, count);
        last_time = currentTime;
    }

    // MuJoCo sensor order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
    // Config state order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
    vector_t jointAngles(ANYMAL_C_NUM_JOINTS);
    vector_t jointVelocities(ANYMAL_C_NUM_JOINTS);

    // LF (MuJoCo 0-2) → state 0-2
    jointAngles[0] = low_state.motor_states[0].q;
    jointAngles[1] = low_state.motor_states[1].q;
    jointAngles[2] = low_state.motor_states[2].q;
    // LH (MuJoCo 6-8) → state 3-5
    jointAngles[3] = low_state.motor_states[6].q;
    jointAngles[4] = low_state.motor_states[7].q;
    jointAngles[5] = low_state.motor_states[8].q;
    // RF (MuJoCo 3-5) → state 6-8
    jointAngles[6] = low_state.motor_states[3].q;
    jointAngles[7] = low_state.motor_states[4].q;
    jointAngles[8] = low_state.motor_states[5].q;
    // RH (MuJoCo 9-11) → state 9-11
    jointAngles[9] = low_state.motor_states[9].q;
    jointAngles[10] = low_state.motor_states[10].q;
    jointAngles[11] = low_state.motor_states[11].q;

    jointVelocities[0] = low_state.motor_states[0].dq;
    jointVelocities[1] = low_state.motor_states[1].dq;
    jointVelocities[2] = low_state.motor_states[2].dq;
    jointVelocities[3] = low_state.motor_states[6].dq;
    jointVelocities[4] = low_state.motor_states[7].dq;
    jointVelocities[5] = low_state.motor_states[8].dq;
    jointVelocities[6] = low_state.motor_states[3].dq;
    jointVelocities[7] = low_state.motor_states[4].dq;
    jointVelocities[8] = low_state.motor_states[5].dq;
    jointVelocities[9] = low_state.motor_states[9].dq;
    jointVelocities[10] = low_state.motor_states[10].dq;
    jointVelocities[11] = low_state.motor_states[11].dq;

    vector4_t baseOrientation;
    baseOrientation[0] = low_state.imu_state.quaternion[1];  // x
    baseOrientation[1] = low_state.imu_state.quaternion[2];  // y
    baseOrientation[2] = low_state.imu_state.quaternion[3];  // z
    baseOrientation[3] = low_state.imu_state.quaternion[0];  // w

    vector3_t baseAngVel;
    baseAngVel[0] = low_state.imu_state.gyroscope[0];
    baseAngVel[1] = low_state.imu_state.gyroscope[1];
    baseAngVel[2] = low_state.imu_state.gyroscope[2];

    vector3_t baseAcc;
    baseAcc[0] = low_state.imu_state.accelerometer[0];
    baseAcc[1] = low_state.imu_state.accelerometer[1];
    baseAcc[2] = low_state.imu_state.accelerometer[2];

    // Contact detection
    constexpr bool useHysteresis = true;
    static std::vector<bool> prevContacts(4, true);
    std::vector<bool> contactFlags(4, true);
    if (low_state.foot_force.size() >= 4) {
        std::vector<double> forces = {static_cast<double>(low_state.foot_force[footIdMap_["LF_FOOT"]]),
                                      static_cast<double>(low_state.foot_force[footIdMap_["RF_FOOT"]]),
                                      static_cast<double>(low_state.foot_force[footIdMap_["LH_FOOT"]]),
                                      static_cast<double>(low_state.foot_force[footIdMap_["RH_FOOT"]])};
        if constexpr (useHysteresis) {
            const double on_threshold = 10.0;
            const double off_threshold = 3.0;
            for (size_t i = 0; i < 4; ++i) {
                contactFlags[i] = prevContacts[i] ? (forces[i] >= off_threshold) : (forces[i] >= on_threshold);
            }
        } else {
            const double contact_threshold = 19.0;
            for (size_t i = 0; i < 4; ++i) {
                contactFlags[i] = forces[i] >= contact_threshold;
            }
        }
        prevContacts = contactFlags;
    }

    static scalar_t lastTime = currentTime;
    scalar_t dt = currentTime - lastTime;
    lastTime = currentTime;

    // Pass joints in config order (LF, LH, RF, RH) — matches Gazebo InekfRosStateSubscriber behavior
    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags, rectifyOrientation_, enable_);

    State state;
    state.x = vector_t::Zero(ANYMAL_C_STATE_DIM);

    const quaternion_t baseQuaternion =
        rectifyOrientation_ ? quaternion_t(baseOrientation) : quaternion_t(estimator_->getBaseOrientation());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    state.x.segment<3>(0) = rpy;

    // Position and velocity
    if (useGroundTruthState_ && low_state.has_position && low_state.has_velocity) {
        // Use ground-truth position/velocity from LowState
        vector3_t gtPos(low_state.position[0], low_state.position[1], low_state.position[2]);
        vector3_t gtVel(low_state.velocity[0], low_state.velocity[1], low_state.velocity[2]);
        state.x.segment<3>(3) = gtPos;
        state.x.segment<3>(9) = R_base_world * gtVel;
    } else {
        // Fall back to EKF with Z-drift correction
        vector3_t basePos = estimator_->getBasePosition();
        if (enableGroundPlaneCorrection_) {
            updateGroundPlaneEstimate();
        }
        if (groundHeightValid_) {
            constexpr double nominalHeight = 0.54;
            constexpr double zCorrectionAlpha = 0.01;
            double expectedZ = groundHeightEstimate_ + nominalHeight;
            int numContacts = 0;
            for (const auto &c : contactFlags) {
                if (c) numContacts++;
            }
            if (numContacts >= 3) {
                basePos[2] = basePos[2] * (1.0 - zCorrectionAlpha) + expectedZ * zCorrectionAlpha;
            }
        } else {
            constexpr double nominalHeight = 0.54;
            constexpr double zCorrectionAlpha = 0.005;
            int numContacts = 0;
            for (const auto &c : contactFlags) {
                if (c) numContacts++;
            }
            if (numContacts >= 3) {
                basePos[2] = basePos[2] * (1.0 - zCorrectionAlpha) + nominalHeight * zCorrectionAlpha;
            }
        }
        state.x.segment<3>(3) = basePos;
        state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();
    }

    if (removeGyroscopeBias_) {
        state.x.segment<3>(6) = baseAngVel - estimator_->getGyroscopeBias();
    } else {
        state.x.segment<3>(6) = baseAngVel;
    }
    state.x.segment<12>(12) = jointAngles;
    state.x.segment<12>(24) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    std::lock_guard<std::mutex> lock(latestStateMutex_);
    state_ = std::move(state);
    initialized_ = true;
}

void AnymalCRobotInterface::publish(std::vector<MotorCommand> commands) {
    static auto last_publish_time = std::chrono::high_resolution_clock::now();
    static int publish_count = 0;
    publish_count++;

    constexpr int PUBLISH_N = 100;
    if (publish_count % PUBLISH_N == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_publish_time).count();
        double rate = (PUBLISH_N * 1000.0) / time_diff;
        TBAI_LOG_INFO(logger_, "Publish frequency: {} Hz (count: {})", rate, publish_count);
        last_publish_time = current_time;
    }

    robot_msgs::MotorCommands motor_commands;
    motor_commands.commands.resize(ANYMAL_C_NUM_JOINTS);

    for (const auto &command : commands) {
        const int motorId = motorIdMap_[command.joint_name];
        motor_commands.commands[motorId].q = command.desired_position;
        motor_commands.commands[motorId].dq = command.desired_velocity;
        motor_commands.commands[motorId].kp = command.kp;
        motor_commands.commands[motorId].kd = command.kd;
        motor_commands.commands[motorId].tau = command.torque_ff;
    }

    lowcmd_publisher->publish(motor_commands);
}

void AnymalCRobotInterface::waitTillInitialized() {
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        TBAI_LOG_INFO_THROTTLE(logger_, 1.0, "Waiting for the ANYmal C robot to initialize...");
    }
    TBAI_LOG_INFO(logger_, "ANYmal C robot initialized");
}

State AnymalCRobotInterface::getLatestState() {
    std::lock_guard<std::mutex> lock(latestStateMutex_);
    return state_;
}

void AnymalCRobotInterface::updateGroundPlaneEstimate() {
    if (!depthSubscriber_) return;

    auto pc = depthSubscriber_->take();
    if (!pc) return;

    double height = estimateGroundHeight(*pc);
    if (std::isfinite(height)) {
        if (!groundHeightValid_) {
            groundHeightEstimate_ = height;
            groundHeightValid_ = true;
        } else {
            // Exponential moving average for smooth updates
            constexpr double alpha = 0.05;
            groundHeightEstimate_ = groundHeightEstimate_ * (1.0 - alpha) + height * alpha;
        }
    }
}

double AnymalCRobotInterface::estimateGroundHeight(const robot_msgs::PointCloud2 &pc) {
    // Extract Z values from pointcloud
    // Find the Z field offset
    int zOffset = -1;
    int pointStep = static_cast<int>(pc.point_step);
    for (const auto &field : pc.fields) {
        if (field.name == "z") {
            zOffset = static_cast<int>(field.offset);
            break;
        }
    }
    if (zOffset < 0 || pointStep == 0 || pc.data.empty()) return std::numeric_limits<double>::quiet_NaN();

    size_t numPoints = pc.data.size() / pointStep;
    if (numPoints < 10) return std::numeric_limits<double>::quiet_NaN();

    // Collect Z values from the pointcloud (in camera frame, which is roughly body-relative)
    // The front_lower depth camera looks downward, so Z in camera frame ~ distance forward,
    // and Y in camera frame ~ distance downward. We need to figure out which axis is "down."
    //
    // For a downward-looking camera, the ground points will have the largest Y values (or Z depending on convention).
    // In the optical frame convention: X=right, Y=down, Z=forward.
    // So ground height in world frame can be estimated from the Y values (which represent depth downward).
    //
    // However, the pointcloud is typically already transformed to the base frame by the sensor bridge.
    // In that case, Z values directly represent height.
    // We use a percentile-based approach: take the 10th percentile of Z values as the ground plane.

    std::vector<float> zValues;
    zValues.reserve(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        float z;
        std::memcpy(&z, &pc.data[i * pointStep + zOffset], sizeof(float));
        if (std::isfinite(z)) {
            zValues.push_back(z);
        }
    }

    if (zValues.size() < 10) return std::numeric_limits<double>::quiet_NaN();

    // Sort and take the 10th percentile as ground estimate
    std::sort(zValues.begin(), zValues.end());
    size_t idx = zValues.size() / 10;  // 10th percentile
    return static_cast<double>(zValues[idx]);
}

}  // namespace tbai
