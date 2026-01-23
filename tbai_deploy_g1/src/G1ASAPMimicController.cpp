#include "tbai_deploy_g1/G1ASAPMimicController.hpp"

#include <algorithm>
#include <cmath>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>

namespace tbai {
namespace g1 {

G1ASAPMimicController::G1ASAPMimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                             const std::string &policyPath, float motionLength,
                                             const std::string &controllerName)
    : stateSubscriberPtr_(stateSubscriberPtr),
      modelLoaded_(false),
      motionLength_(motionLength),
      motionStartTime_(0.0f),
      motionPhase_(0.0f),
      motionActive_(false),
      motionComplete_(false),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {} (ASAP Mimic)", controllerName_);

    // Load configuration
    rate_ = tbai::fromGlobalConfig<float>("g1_asap_mimic_controller/rate", 50.0f);
    actionScale_ = tbai::fromGlobalConfig<float>("g1_asap_mimic_controller/action_scale", 0.25f);
    actionClip_ = tbai::fromGlobalConfig<float>("g1_asap_mimic_controller/action_clip", 100.0f);

    angVelScale_ = tbai::fromGlobalConfig<float>("g1_asap_mimic_controller/ang_vel_scale", 0.25f);
    dofVelScale_ = tbai::fromGlobalConfig<float>("g1_asap_mimic_controller/dof_vel_scale", 0.05f);

    // Initialize joint mapping (23 mimic DOFs to 29 G1 DOFs)
    initJointMapping();

    // Load default joint positions for 23-DOF mimic
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_mimic_controller/default_joint_pos");
    if (defaultJointPosVec.size() != ASAP_MIMIC_NUM_JOINTS) {
        throw std::runtime_error("Expected 23 default_joint_pos values for ASAP Mimic");
    }
    defaultJointPos23_ = vector_t(ASAP_MIMIC_NUM_JOINTS);
    for (int i = 0; i < ASAP_MIMIC_NUM_JOINTS; ++i) {
        defaultJointPos23_[i] = defaultJointPosVec[i];
    }

    // Load full 29-DOF default positions for motor commands
    auto defaultJointPos29Vec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_mimic_controller/default_joint_pos_full");
    if (defaultJointPos29Vec.size() != G1_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 default_joint_pos_full values for ASAP Mimic");
    }
    defaultJointPos29_ = vector_t(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        defaultJointPos29_[i] = defaultJointPos29Vec[i];
    }

    // Load stiffness and damping (29 joints)
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_mimic_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_mimic_controller/damping");
    if (stiffnessVec.size() != G1_NUM_JOINTS || dampingVec.size() != G1_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 stiffness/damping values for ASAP Mimic");
    }
    stiffness_ = vector_t(G1_NUM_JOINTS);
    damping_ = vector_t(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Initialize vectors
    lastAction_ = vector_t::Zero(ASAP_MIMIC_NUM_JOINTS);
    action_ = vector_t::Zero(ASAP_MIMIC_NUM_JOINTS);
    observation_ = vector_t::Zero(ASAP_MIMIC_TOTAL_OBS_SIZE);

    // Initialize current observation components
    currentAngVel_ = vector_t::Zero(3);
    currentGravity_ = vector_t::Zero(3);
    currentDofPos_ = vector_t::Zero(ASAP_MIMIC_NUM_JOINTS);
    currentDofVel_ = vector_t::Zero(ASAP_MIMIC_NUM_JOINTS);
    currentPhase_ = 0.0;

    // Initialize history buffers with zeros
    for (int i = 0; i < ASAP_MIMIC_HISTORY_LENGTH; ++i) {
        historyActions_.push_back(vector_t::Zero(ASAP_MIMIC_NUM_JOINTS));
        historyAngVel_.push_back(vector_t::Zero(3));
        historyDofPos_.push_back(vector_t::Zero(ASAP_MIMIC_NUM_JOINTS));
        historyDofVel_.push_back(vector_t::Zero(ASAP_MIMIC_NUM_JOINTS));
        historyGravity_.push_back(vector_t::Zero(3));
        historyPhase_.push_back(0.0);
    }

    // Initialize ONNX Runtime model
    initOnnxModel(policyPath);

    TBAI_LOG_INFO(logger_, "{} initialized (motion_length={:.2f}s, obs_size={})", controllerName_, motionLength_,
                  ASAP_MIMIC_TOTAL_OBS_SIZE);
}

G1ASAPMimicController::~G1ASAPMimicController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
}

void G1ASAPMimicController::initJointMapping() {
    // g1_29dof_anneal_23dof DOF mask from ASAP config:
    // [1,1,1,1,1,1, 1,1,1,1,1,1, 1,1,1, 1,1,1,1,0,0,0, 1,1,1,1,0,0,0]
    // This means wrist joints (indices 19,20,21 and 26,27,28 in 29-DOF) are locked
    //
    // 23-DOF mimic order maps to 29-DOF G1:
    // Mimic 0-5:  left leg (G1 0-5)
    // Mimic 6-11: right leg (G1 6-11)
    // Mimic 12-14: waist (G1 12-14)
    // Mimic 15-18: left arm without wrists (G1 15-18)
    // Mimic 19-22: right arm without wrists (G1 22-25)

    dofMask_ = {
        true, true, true, true, true, true,     // left leg (0-5)
        true, true, true, true, true, true,     // right leg (6-11)
        true, true, true,                        // waist (12-14)
        true, true, true, true,                  // left arm (15-18)
        false, false, false,                     // left wrist (19-21) - LOCKED
        true, true, true, true,                  // right arm (22-25)
        false, false, false                      // right wrist (26-28) - LOCKED
    };

    // Build mapping from 23-DOF mimic index to 29-DOF G1 index
    mimicToG1Mapping_.clear();
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        if (dofMask_[i]) {
            mimicToG1Mapping_.push_back(i);
        }
    }

    if (mimicToG1Mapping_.size() != ASAP_MIMIC_NUM_JOINTS) {
        throw std::runtime_error("DOF mask does not produce 23 active joints");
    }

    TBAI_LOG_INFO(logger_, "Joint mapping initialized: {} mimic DOFs -> {} G1 DOFs", ASAP_MIMIC_NUM_JOINTS, G1_NUM_JOINTS);
}

void G1ASAPMimicController::initOnnxModel(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    try {
        ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1ASAPMimicController");
        ortSessionOptions_ = std::make_unique<Ort::SessionOptions>();
        ortSessionOptions_->SetIntraOpNumThreads(1);
        ortSessionOptions_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), *ortSessionOptions_);

        Ort::AllocatorWithDefaultOptions allocator;
        auto inputNameAllocated = ortSession_->GetInputNameAllocated(0, allocator);
        auto outputNameAllocated = ortSession_->GetOutputNameAllocated(0, allocator);
        inputName_ = inputNameAllocated.get();
        outputName_ = outputNameAllocated.get();

        modelLoaded_ = true;
        TBAI_LOG_INFO(logger_, "ONNX model loaded (input: {}, output: {})", inputName_, outputName_);
    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Failed to load ONNX model: {}", e.what());
        throw std::runtime_error("Failed to load ONNX model: " + std::string(e.what()));
    }
}

void G1ASAPMimicController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();

    // Update motion phase
    if (motionActive_ && !motionComplete_) {
        float elapsed = static_cast<float>(currentTime) - motionStartTime_;
        motionPhase_ = elapsed / motionLength_;

        if (motionPhase_ >= 1.0f) {
            motionPhase_ = 1.0f;
            motionComplete_ = true;
            TBAI_LOG_INFO(logger_, "Motion complete");
        }
    }

    currentPhase_ = motionPhase_;
}

vector_t G1ASAPMimicController::getProjectedGravity(const vector4_t &quat) const {
    // Same as ASAP controller - quat format [x, y, z, w]
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

void G1ASAPMimicController::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Get state components
    vector3_t baseAngVel = state_.x.segment<3>(6);

    // Get quaternion for gravity projection
    vector4_t quat;
    auto *g1Interface = dynamic_cast<G1RobotInterface *>(stateSubscriberPtr_.get());
    if (g1Interface) {
        quat = g1Interface->getBaseQuaternion();
    } else {
        double roll = state_.x(0);
        double pitch = state_.x(1);
        double yaw = state_.x(2);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        quat(0) = sr * cp * cy - cr * sp * sy;
        quat(1) = cr * sp * cy + sr * cp * sy;
        quat(2) = cr * cp * sy - sr * sp * cy;
        quat(3) = cr * cp * cy + sr * sp * sy;
    }

    vector_t gravity = getProjectedGravity(quat);

    // Get joint state (29 DOF) and map to 23 DOF mimic space
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Store current observation components
    currentAngVel_ = baseAngVel;
    currentGravity_ = gravity;

    // Map 29-DOF to 23-DOF using dof mask
    for (int i = 0; i < ASAP_MIMIC_NUM_JOINTS; ++i) {
        int g1Idx = mimicToG1Mapping_[i];
        currentDofPos_[i] = jointPos[g1Idx] - defaultJointPos23_[i];
        currentDofVel_[i] = jointVel[g1Idx];
    }

    // === Build full observation (380 dims) ===
    int idx = 0;

    // ==== CURRENT OBSERVATION (72 dims) ====

    // 1. last_action (23 dims)
    observation_.segment(idx, ASAP_MIMIC_NUM_JOINTS) = lastAction_;
    idx += ASAP_MIMIC_NUM_JOINTS;

    // 2. base_ang_vel * 0.25 (3 dims)
    observation_.segment<3>(idx) = currentAngVel_ * angVelScale_;
    idx += 3;

    // 3. dof_pos - default (23 dims)
    observation_.segment(idx, ASAP_MIMIC_NUM_JOINTS) = currentDofPos_;
    idx += ASAP_MIMIC_NUM_JOINTS;

    // 4. dof_vel * 0.05 (23 dims)
    observation_.segment(idx, ASAP_MIMIC_NUM_JOINTS) = currentDofVel_ * dofVelScale_;
    idx += ASAP_MIMIC_NUM_JOINTS;

    // ==== HISTORY (304 dims, ALPHABETICALLY sorted) ====

    // 1. actions (23 * 4 = 92)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_MIMIC_NUM_JOINTS, ASAP_MIMIC_NUM_JOINTS) = historyActions_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH * ASAP_MIMIC_NUM_JOINTS;

    // 2. base_ang_vel (3 * 4 = 12)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyAngVel_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH * 3;

    // 3. dof_pos (23 * 4 = 92)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_MIMIC_NUM_JOINTS, ASAP_MIMIC_NUM_JOINTS) = historyDofPos_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH * ASAP_MIMIC_NUM_JOINTS;

    // 4. dof_vel (23 * 4 = 92)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_MIMIC_NUM_JOINTS, ASAP_MIMIC_NUM_JOINTS) = historyDofVel_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH * ASAP_MIMIC_NUM_JOINTS;

    // 5. projected_gravity (3 * 4 = 12)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyGravity_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH * 3;

    // 6. ref_motion_phase (1 * 4 = 4)
    for (int t = 0; t < ASAP_MIMIC_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyPhase_[t];
    }
    idx += ASAP_MIMIC_HISTORY_LENGTH;

    // ==== POST-HISTORY (4 dims) ====

    // 1. projected_gravity (3 dims)
    observation_.segment<3>(idx) = currentGravity_;
    idx += 3;

    // 2. ref_motion_phase (1 dim)
    observation_[idx] = currentPhase_;
    idx += 1;

    // Sanity check
    if (idx != ASAP_MIMIC_TOTAL_OBS_SIZE) {
        TBAI_LOG_ERROR(logger_, "Observation size mismatch: {} != {}", idx, ASAP_MIMIC_TOTAL_OBS_SIZE);
    }

    // Clip observation
    const scalar_t clipObs = 100.0;
    for (int i = 0; i < ASAP_MIMIC_TOTAL_OBS_SIZE; ++i) {
        observation_[i] = std::clamp(observation_[i], -clipObs, clipObs);
    }
}

void G1ASAPMimicController::updateHistory() {
    // Push current observations to history (newest at index 0)
    historyActions_.pop_back();
    historyActions_.push_front(lastAction_);

    historyAngVel_.pop_back();
    historyAngVel_.push_front(currentAngVel_ * angVelScale_);

    historyDofPos_.pop_back();
    historyDofPos_.push_front(currentDofPos_);

    historyDofVel_.pop_back();
    historyDofVel_.push_front(currentDofVel_ * dofVelScale_);

    historyGravity_.pop_back();
    historyGravity_.push_front(currentGravity_);

    historyPhase_.pop_back();
    historyPhase_.push_front(currentPhase_);
}

void G1ASAPMimicController::runInference() {
    if (!modelLoaded_) {
        TBAI_LOG_ERROR(logger_, "Model not loaded!");
        return;
    }

    try {
        std::vector<int64_t> inputShape = {1, ASAP_MIMIC_TOTAL_OBS_SIZE};
        std::vector<float> inputData(observation_.data(), observation_.data() + observation_.size());

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor =
            Ort::Value::CreateTensor<float>(memoryInfo, inputData.data(), inputData.size(), inputShape.data(), 2);

        const char *inputNames[] = {inputName_.c_str()};
        const char *outputNames[] = {outputName_.c_str()};

        auto outputTensors = ortSession_->Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);

        float *outputData = outputTensors[0].GetTensorMutableData<float>();

        // Store raw action for next step's observation
        for (int i = 0; i < ASAP_MIMIC_NUM_JOINTS; ++i) {
            lastAction_[i] = static_cast<scalar_t>(outputData[i]);
        }

        // Apply clipping and scaling
        for (int i = 0; i < ASAP_MIMIC_NUM_JOINTS; ++i) {
            scalar_t rawAction = lastAction_[i];
            action_[i] = std::clamp(rawAction, -actionClip_, actionClip_) * actionScale_;
        }

    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Inference failed: {}", e.what());
    }
}

std::vector<tbai::MotorCommand> G1ASAPMimicController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();
    updateHistory();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    // Create commands for all 29 G1 joints
    int mimicIdx = 0;
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        cmd.joint_name = jointNames_[i];

        if (dofMask_[i]) {
            // Active joint: use policy output
            cmd.desired_position = action_[mimicIdx] + defaultJointPos29_[i];
            mimicIdx++;
        } else {
            // Locked wrist joint: hold at default
            cmd.desired_position = defaultJointPos29_[i];
        }

        cmd.desired_velocity = 0.0;
        cmd.kp = stiffness_[i];
        cmd.kd = damping_[i];
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    return commands;
}

bool G1ASAPMimicController::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1ASAPMimicController" ||
           controllerType == "G1ASAPMimic" || controllerType == "g1_asap_mimic";
}

void G1ASAPMimicController::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1ASAPMimicController::ok() const {
    return true;
}

bool G1ASAPMimicController::checkStability() const {
    const scalar_t maxAngle = 1.0;
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

void G1ASAPMimicController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset actions
    lastAction_.setZero();
    action_.setZero();

    // Reset motion tracking
    motionStartTime_ = static_cast<float>(currentTime);
    motionPhase_ = 0.0f;
    motionActive_ = true;
    motionComplete_ = false;

    // Clear history buffers
    for (int i = 0; i < ASAP_MIMIC_HISTORY_LENGTH; ++i) {
        historyActions_[i].setZero();
        historyAngVel_[i].setZero();
        historyDofPos_[i].setZero();
        historyDofVel_[i].setZero();
        historyGravity_[i].setZero();
        historyPhase_[i] = 0.0;
    }

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    TBAI_LOG_INFO(logger_, "{} ready (motion_length={:.2f}s)", controllerName_, motionLength_);
}

}  // namespace g1
}  // namespace tbai
