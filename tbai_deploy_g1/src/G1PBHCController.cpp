#include "tbai_deploy_g1/G1PBHCController.hpp"

#include <algorithm>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>

namespace tbai {
namespace g1 {

G1PBHCController::G1PBHCController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                   const std::string &policyPath, const std::string &motionFilePath, float timeStart,
                                   float timeEnd, const std::string &controllerName)
    : stateSubscriberPtr_(stateSubscriberPtr),
      modelLoaded_(false),
      timeStart_(timeStart),
      currentMotionTime_(0.0f),
      motionActive_(false),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {} (PBHC)", controllerName_);

    // Load configuration
    rate_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/rate", 50.0f);
    actionScale_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/action_scale", 0.25f);
    actionClip_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/action_clip", 100.0f);

    angVelScale_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/ang_vel_scale", 0.25f);
    dofPosScale_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/dof_pos_scale", 1.0f);
    dofVelScale_ = tbai::fromGlobalConfig<float>("g1_pbhc_controller/dof_vel_scale", 0.05f);

    // Load default joint positions (23 PBHC joints)
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_pbhc_controller/default_joint_pos");
    if (defaultJointPosVec.size() != PBHC_NUM_JOINTS) {
        throw std::runtime_error("Expected 23 default_joint_pos values for PBHC");
    }
    defaultJointPos_ = vector_t(PBHC_NUM_JOINTS);
    for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
    }

    // Load stiffness and damping (23 joints)
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_pbhc_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_pbhc_controller/damping");
    if (stiffnessVec.size() != PBHC_NUM_JOINTS || dampingVec.size() != PBHC_NUM_JOINTS) {
        throw std::runtime_error("Expected 23 stiffness/damping values for PBHC");
    }
    stiffness_ = vector_t(PBHC_NUM_JOINTS);
    damping_ = vector_t(PBHC_NUM_JOINTS);
    for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    // Joint mapping from 23-DOF PBHC order to 29-DOF G1 DDS order
    // PBHC order (23 DOF, locked wrists):
    // 0-5: left leg (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
    // 6-11: right leg
    // 12-14: waist (yaw, roll, pitch)
    // 15-18: left arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow)
    // 19-22: right arm
    //
    // G1 DDS order (29 DOF):
    // 0-5: left leg
    // 6-11: right leg
    // 12-14: waist
    // 15-21: left arm with wrists (shoulder_pitch, roll, yaw, elbow, wrist_roll, pitch, yaw)
    // 22-28: right arm with wrists
    jointMapping_ = {
        0,  1,  2,  3,  4,  5,   // left leg (0-5) -> (0-5)
        6,  7,  8,  9,  10, 11,  // right leg (6-11) -> (6-11)
        12, 13, 14,              // waist (12-14) -> (12-14)
        15, 16, 17, 18,          // left arm (15-18) -> (15, 16, 17, 18)
        22, 23, 24, 25           // right arm (19-22) -> (22, 23, 24, 25)
    };

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load motion data
    motionLoader_ = std::make_unique<PBHCMotionLoader>(motionFilePath);
    TBAI_LOG_INFO(logger_, "Loaded motion file with {} frames, duration: {:.2f}s", motionLoader_->getNumFrames(),
                  motionLoader_->getDuration());

    // Set time end (use full duration if -1)
    timeEnd_ = (timeEnd < 0) ? motionLoader_->getDuration() : timeEnd;
    timeEnd_ = std::clamp(timeEnd_, 0.0f, motionLoader_->getDuration());
    timeStart_ = std::clamp(timeStart_, 0.0f, timeEnd_);

    motionLoader_->setTimeRange(timeStart_, timeEnd_);

    TBAI_LOG_INFO(logger_, "Motion playback range: {:.2f}s - {:.2f}s", timeStart_, timeEnd_);

    // Initialize vectors
    lastAction_ = vector_t::Zero(PBHC_NUM_JOINTS);
    action_ = vector_t::Zero(PBHC_NUM_JOINTS);
    observation_ = vector_t::Zero(PBHC_TOTAL_OBS_SIZE);

    // Initialize current observation components
    currentAngVel_ = vector_t::Zero(3);
    currentGravity_ = vector_t::Zero(3);
    currentDofPos_ = vector_t::Zero(PBHC_NUM_JOINTS);
    currentDofVel_ = vector_t::Zero(PBHC_NUM_JOINTS);
    currentActions_ = vector_t::Zero(PBHC_NUM_JOINTS);
    currentPhase_ = 0.0;

    // Initialize history buffers with zeros
    for (int i = 0; i < PBHC_HISTORY_LENGTH; ++i) {
        historyActions_.push_back(vector_t::Zero(PBHC_NUM_JOINTS));
        historyAngVel_.push_back(vector_t::Zero(3));
        historyDofPos_.push_back(vector_t::Zero(PBHC_NUM_JOINTS));
        historyDofVel_.push_back(vector_t::Zero(PBHC_NUM_JOINTS));
        historyGravity_.push_back(vector_t::Zero(3));
        historyPhase_.push_back(0.0);
    }

    // Initialize ONNX Runtime model
    initOnnxModel(policyPath);

    TBAI_LOG_INFO(logger_, "{} initialized successfully (obs_size={})", controllerName_, PBHC_TOTAL_OBS_SIZE);
}

G1PBHCController::~G1PBHCController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
}

void G1PBHCController::initOnnxModel(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    try {
        // Create ONNX Runtime environment
        ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1PBHCController");

        // Create session options
        ortSessionOptions_ = std::make_unique<Ort::SessionOptions>();
        ortSessionOptions_->SetIntraOpNumThreads(1);
        ortSessionOptions_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        // Create session
        ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), *ortSessionOptions_);

        // Get input/output names
        Ort::AllocatorWithDefaultOptions allocator;
        auto inputNameAllocated = ortSession_->GetInputNameAllocated(0, allocator);
        auto outputNameAllocated = ortSession_->GetOutputNameAllocated(0, allocator);
        inputName_ = inputNameAllocated.get();
        outputName_ = outputNameAllocated.get();

        modelLoaded_ = true;
        TBAI_LOG_INFO(logger_, "ONNX model loaded successfully (input: {}, output: {})", inputName_, outputName_);
    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Failed to load ONNX model: {}", e.what());
        throw std::runtime_error("Failed to load ONNX model: " + std::string(e.what()));
    }
}

void G1PBHCController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();

    // Update motion time
    if (motionActive_) {
        motionLoader_->advance(static_cast<float>(dt));
        currentMotionTime_ = motionLoader_->getCurrentTime() - timeStart_;
    }
}

vector_t G1PBHCController::getProjectedGravity(const vector4_t &quat) const {
    double qx = quat(0);
    double qy = quat(1);
    double qz = quat(2);
    double qw = quat(3);

    vector_t result(3);
    result(0) = 2.0 * (qw * qy - qx * qz);
    result(1) = -2.0 * (qw * qx + qy * qz);
    result(2) = 1.0 - 2.0 * (qw * qw + qz * qz);

    return result;
}

void G1PBHCController::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Get state components
    vector3_t baseAngVel = state_.x.segment<3>(6);

    // Get quaternion for gravity projection
    vector4_t quat;
    auto *g1Interface = dynamic_cast<G1RobotInterface *>(stateSubscriberPtr_.get());
    if (g1Interface) {
        quat = g1Interface->getBaseQuaternion();
    } else {
        // Fallback: convert RPY to quaternion
        double roll = state_.x(0);
        double pitch = state_.x(1);
        double yaw = state_.x(2);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        quat(0) = sr * cp * cy - cr * sp * sy;  // x
        quat(1) = cr * sp * cy + sr * cp * sy;  // y
        quat(2) = cr * cp * sy - sr * sp * cy;  // z
        quat(3) = cr * cp * cy + sr * sp * sy;  // w
    }

    vector_t gravity = getProjectedGravity(quat);

    // Map from 29-DOF G1 state to 23-DOF PBHC order
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Store current observation components (unscaled for history)
    currentAngVel_ = baseAngVel;
    currentGravity_ = gravity;
    for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
        int g1Idx = jointMapping_[i];
        currentDofPos_[i] = jointPos[g1Idx] - defaultJointPos_[i];
        currentDofVel_[i] = jointVel[g1Idx];
    }
    currentActions_ = lastAction_;
    currentPhase_ = motionLoader_->getRefMotionPhase();

    // === Build full observation (380 dims) ===
    // CRITICAL: PBHC concatenates observations in ALPHABETICAL order!
    // Order: actions, base_ang_vel, dof_pos, dof_vel, history_actor, projected_gravity, ref_motion_phase
    int idx = 0;

    // 1. actions (23 dims, current)
    observation_.segment(idx, PBHC_NUM_JOINTS) = currentActions_;
    idx += PBHC_NUM_JOINTS;

    // 2. base_ang_vel (3 dims, current, scaled)
    observation_.segment<3>(idx) = currentAngVel_ * angVelScale_;
    idx += 3;

    // 3. dof_pos (23 dims, current, scaled)
    observation_.segment(idx, PBHC_NUM_JOINTS) = currentDofPos_ * dofPosScale_;
    idx += PBHC_NUM_JOINTS;

    // 4. dof_vel (23 dims, current, scaled)
    observation_.segment(idx, PBHC_NUM_JOINTS) = currentDofVel_ * dofVelScale_;
    idx += PBHC_NUM_JOINTS;

    // 5. history_actor (304 dims, sorted alphabetically by component)
    // Components: actions, base_ang_vel, dof_pos, dof_vel, projected_gravity, ref_motion_phase

    // history_actions (4 * 23 = 92)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * PBHC_NUM_JOINTS, PBHC_NUM_JOINTS) = historyActions_[t];
    }
    idx += PBHC_HISTORY_LENGTH * PBHC_NUM_JOINTS;

    // history_base_ang_vel (4 * 3 = 12, scaled)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyAngVel_[t] * angVelScale_;
    }
    idx += PBHC_HISTORY_LENGTH * 3;

    // history_dof_pos (4 * 23 = 92, scaled)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * PBHC_NUM_JOINTS, PBHC_NUM_JOINTS) = historyDofPos_[t] * dofPosScale_;
    }
    idx += PBHC_HISTORY_LENGTH * PBHC_NUM_JOINTS;

    // history_dof_vel (4 * 23 = 92, scaled)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * PBHC_NUM_JOINTS, PBHC_NUM_JOINTS) = historyDofVel_[t] * dofVelScale_;
    }
    idx += PBHC_HISTORY_LENGTH * PBHC_NUM_JOINTS;

    // history_projected_gravity (4 * 3 = 12)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyGravity_[t];
    }
    idx += PBHC_HISTORY_LENGTH * 3;

    // history_ref_motion_phase (4 * 1 = 4)
    for (int t = 0; t < PBHC_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyPhase_[t];
    }
    idx += PBHC_HISTORY_LENGTH;

    // 6. projected_gravity (3 dims, current)
    observation_.segment<3>(idx) = currentGravity_;
    idx += 3;

    // 7. ref_motion_phase (1 dim, current)
    observation_[idx] = currentPhase_;
    idx += 1;

    // Clip full observation (current + history)
    const scalar_t clipObs = 100.0;
    for (int i = 0; i < PBHC_TOTAL_OBS_SIZE; ++i) {
        observation_[i] = std::clamp(observation_[i], -clipObs, clipObs);
    }
}

void G1PBHCController::updateHistory() {
    // Push current observations to history (newest first - position 0 is most recent)
    // This matches PBHC's HistoryHandler which stores newest at index 0
    historyActions_.pop_back();
    historyActions_.push_front(currentActions_);

    historyAngVel_.pop_back();
    historyAngVel_.push_front(currentAngVel_);

    historyDofPos_.pop_back();
    historyDofPos_.push_front(currentDofPos_);

    historyDofVel_.pop_back();
    historyDofVel_.push_front(currentDofVel_);

    historyGravity_.pop_back();
    historyGravity_.push_front(currentGravity_);

    historyPhase_.pop_back();
    historyPhase_.push_front(currentPhase_);
}

void G1PBHCController::runInference() {
    if (!modelLoaded_) {
        TBAI_LOG_ERROR(logger_, "Model not loaded!");
        return;
    }

    try {
        // Prepare input tensor
        std::vector<int64_t> inputShape = {1, PBHC_TOTAL_OBS_SIZE};
        std::vector<float> inputData(observation_.data(), observation_.data() + observation_.size());

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor =
            Ort::Value::CreateTensor<float>(memoryInfo, inputData.data(), inputData.size(), inputShape.data(), 2);

        // Run inference
        const char *inputNames[] = {inputName_.c_str()};
        const char *outputNames[] = {outputName_.c_str()};

        auto outputTensors = ortSession_->Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);

        // Get output
        float *outputData = outputTensors[0].GetTensorMutableData<float>();

        // Store raw action for next step's observation (BEFORE clipping/scaling)
        for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
            lastAction_[i] = static_cast<scalar_t>(outputData[i]);
        }

        // Apply clipping and scaling for motor commands
        for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
            scalar_t rawAction = lastAction_[i];
            action_[i] = std::clamp(rawAction, -actionClip_, actionClip_) * actionScale_;
        }

    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Inference failed: {}", e.what());
    }
}

std::vector<tbai::MotorCommand> G1PBHCController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();
    updateHistory();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    // Create commands for all 29 G1 joints
    // For the 23 PBHC joints, use computed actions
    // For the 6 wrist joints (not controlled by PBHC), set to zero position with low stiffness

    // First, fill all 29 joints with zero defaults
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        cmd.joint_name = jointNames_[i];
        cmd.desired_position = 0.0;  // Will be overwritten for controlled joints
        cmd.desired_velocity = 0.0;
        cmd.kp = 20.0;  // Low stiffness for uncontrolled wrists
        cmd.kd = 1.0;
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    // Now set the 23 PBHC-controlled joints
    for (int i = 0; i < PBHC_NUM_JOINTS; ++i) {
        int g1Idx = jointMapping_[i];
        commands[g1Idx].desired_position = action_[i] + defaultJointPos_[i];
        commands[g1Idx].kp = stiffness_[i];
        commands[g1Idx].kd = damping_[i];
    }

    return commands;
}

bool G1PBHCController::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1PBHCController" || controllerType == "g1_pbhc" ||
           controllerType == "pbhc";
}

void G1PBHCController::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1PBHCController::ok() const {
    return true;
}

bool G1PBHCController::checkStability() const {
    const scalar_t maxAngle = 1.0;  // ~57 degrees
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

bool G1PBHCController::isMotionComplete() const {
    return motionLoader_->isComplete();
}

void G1PBHCController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset actions
    lastAction_.setZero();
    action_.setZero();

    // Clear history buffers
    for (int i = 0; i < PBHC_HISTORY_LENGTH; ++i) {
        historyActions_[i].setZero();
        historyAngVel_[i].setZero();
        historyDofPos_[i].setZero();
        historyDofVel_[i].setZero();
        historyGravity_[i].setZero();
        historyPhase_[i] = 0.0;
    }

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    // Reset motion loader
    motionLoader_->reset();

    // Reset motion time
    currentMotionTime_ = 0.0f;
    motionActive_ = true;

    TBAI_LOG_INFO(logger_, "Motion playback started from {:.2f}s", timeStart_);
}

}  // namespace g1
}  // namespace tbai
