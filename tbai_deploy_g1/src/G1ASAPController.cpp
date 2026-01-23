#include "tbai_deploy_g1/G1ASAPController.hpp"

#include <algorithm>
#include <cmath>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>

namespace tbai {
namespace g1 {

G1ASAPController::G1ASAPController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                                   const std::string &policyPath, const std::string &controllerName)
    : stateSubscriberPtr_(stateSubscriberPtr),
      refVelGenPtr_(refVelGenPtr),
      modelLoaded_(false),
      phaseTime_(0.0),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {} (ASAP Decoupled Locomotion)", controllerName_);

    // Load configuration
    rate_ = tbai::fromGlobalConfig<float>("g1_asap_controller/rate", 50.0f);
    actionScale_ = tbai::fromGlobalConfig<float>("g1_asap_controller/action_scale", 0.25f);
    actionClip_ = tbai::fromGlobalConfig<float>("g1_asap_controller/action_clip", 100.0f);

    angVelScale_ = tbai::fromGlobalConfig<float>("g1_asap_controller/ang_vel_scale", 0.25f);
    dofVelScale_ = tbai::fromGlobalConfig<float>("g1_asap_controller/dof_vel_scale", 0.05f);
    baseHeightScale_ = tbai::fromGlobalConfig<float>("g1_asap_controller/base_height_scale", 2.0f);
    gaitPeriod_ = tbai::fromGlobalConfig<float>("g1_asap_controller/gait_period", 0.9f);
    defaultBaseHeight_ = tbai::fromGlobalConfig<float>("g1_asap_controller/default_base_height", 0.78f);

    // Load default joint positions (29 joints)
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_controller/default_joint_pos");
    if (defaultJointPosVec.size() != ASAP_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 default_joint_pos values for ASAP");
    }
    defaultJointPos_ = vector_t(ASAP_NUM_JOINTS);
    for (int i = 0; i < ASAP_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
    }

    // Load upper body reference positions (17 joints)
    auto defaultUpperBodyRefVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_controller/default_upper_body_ref");
    if (defaultUpperBodyRefVec.size() != ASAP_UPPER_BODY_JOINTS) {
        throw std::runtime_error("Expected 17 default_upper_body_ref values for ASAP");
    }
    refUpperDofPos_ = vector_t(ASAP_UPPER_BODY_JOINTS);
    for (int i = 0; i < ASAP_UPPER_BODY_JOINTS; ++i) {
        refUpperDofPos_[i] = defaultUpperBodyRefVec[i];
    }

    // Load stiffness and damping (29 joints)
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_asap_controller/damping");
    if (stiffnessVec.size() != ASAP_NUM_JOINTS || dampingVec.size() != ASAP_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 stiffness/damping values for ASAP");
    }
    stiffness_ = vector_t(ASAP_NUM_JOINTS);
    damping_ = vector_t(ASAP_NUM_JOINTS);
    for (int i = 0; i < ASAP_NUM_JOINTS; ++i) {
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Initialize vectors
    lastPolicyAction_ = vector_t::Zero(ASAP_LOWER_BODY_JOINTS);
    action_ = vector_t::Zero(ASAP_NUM_JOINTS);
    observation_ = vector_t::Zero(ASAP_TOTAL_OBS_SIZE);

    // Initialize current observation components
    currentAngVel_ = vector_t::Zero(3);
    currentGravity_ = vector_t::Zero(3);
    currentDofPos_ = vector_t::Zero(ASAP_NUM_JOINTS);
    currentDofVel_ = vector_t::Zero(ASAP_NUM_JOINTS);
    currentLinVelCmd_ = vector_t::Zero(2);
    currentAngVelCmd_ = 0.0;
    currentStandCmd_ = 0.0;
    currentBaseHeightCmd_ = defaultBaseHeight_;
    currentCosPhase_ = 1.0;  // cos(0) = 1
    currentSinPhase_ = 0.0;  // sin(0) = 0

    // Initialize history buffers with zeros
    for (int i = 0; i < ASAP_HISTORY_LENGTH; ++i) {
        historyActions_.push_back(vector_t::Zero(ASAP_LOWER_BODY_JOINTS));
        historyAngVel_.push_back(vector_t::Zero(3));
        historyAngVelCmd_.push_back(0.0);
        historyBaseHeightCmd_.push_back(defaultBaseHeight_ * baseHeightScale_);
        historyLinVelCmd_.push_back(vector_t::Zero(2));
        historyStandCmd_.push_back(0.0);
        historyCosPhase_.push_back(1.0);
        historyDofPos_.push_back(vector_t::Zero(ASAP_NUM_JOINTS));
        historyDofVel_.push_back(vector_t::Zero(ASAP_NUM_JOINTS));
        historyGravity_.push_back(vector_t::Zero(3));
        historyRefUpperDofPos_.push_back(refUpperDofPos_);
        historySinPhase_.push_back(0.0);
    }

    // Initialize ONNX Runtime model
    initOnnxModel(policyPath);

    TBAI_LOG_INFO(logger_, "{} initialized successfully (obs_size={})", controllerName_, ASAP_TOTAL_OBS_SIZE);
}

G1ASAPController::~G1ASAPController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
}

void G1ASAPController::initOnnxModel(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    try {
        // Create ONNX Runtime environment
        ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1ASAPController");

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

void G1ASAPController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();

    // Update phase based on dt and stand command
    updatePhase(dt);
}

void G1ASAPController::updatePhase(scalar_t dt) {
    // Phase only advances when stand_command is active (robot is walking)
    // This matches ASAP's behavior: phase_time = cur_time * stand_command
    if (currentStandCmd_ > 0.5) {
        phaseTime_ += dt;
    }
    // Wrap phase time to [0, gait_period)
    while (phaseTime_ >= gaitPeriod_) {
        phaseTime_ -= gaitPeriod_;
    }

    scalar_t normalizedPhase = phaseTime_ / gaitPeriod_;
    currentSinPhase_ = std::sin(2.0 * M_PI * normalizedPhase);
    currentCosPhase_ = std::cos(2.0 * M_PI * normalizedPhase);
}

vector_t G1ASAPController::getProjectedGravity(const vector4_t &quat) const {
    // ASAP uses quaternion format [w, x, y, z] in quat_rotate_inverse
    // But G1RobotInterface returns [x, y, z, w]
    // We need to handle this properly
    //
    // quat_rotate_inverse formula for gravity [0, 0, -1]:
    // q_w = w, q_vec = [x, y, z]
    // a = v * (2*w^2 - 1)
    // b = cross(q_vec, v) * 2*w
    // c = q_vec * 2*dot(q_vec, v)
    // result = a - b + c
    //
    // For G1RobotInterface format [x, y, z, w]:
    double qx = quat(0);
    double qy = quat(1);
    double qz = quat(2);
    double qw = quat(3);

    // For gravity [0, 0, -1]:
    vector_t result(3);
    result(0) = 2.0 * (qw * qy - qx * qz);
    result(1) = -2.0 * (qw * qx + qy * qz);
    result(2) = 1.0 - 2.0 * (qw * qw + qz * qz);

    return result;
}

void G1ASAPController::buildObservation(scalar_t currentTime, scalar_t dt) {
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

    // Get joint state (29 DOF)
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Store current observation components (unscaled for history)
    currentAngVel_ = baseAngVel;
    currentGravity_ = gravity;
    for (int i = 0; i < ASAP_NUM_JOINTS; ++i) {
        currentDofPos_[i] = jointPos[i] - defaultJointPos_[i];
        currentDofVel_[i] = jointVel[i];
    }

    // Get velocity commands from reference generator
    auto refVel = refVelGenPtr_->getReferenceVelocity(currentTime, dt);
    currentLinVelCmd_[0] = std::clamp(refVel.velocity_x, -0.5, 1.0);
    currentLinVelCmd_[1] = std::clamp(refVel.velocity_y, -0.5, 0.5);
    currentAngVelCmd_ = std::clamp(refVel.yaw_rate, -1.0, 1.0);

    // Stand command: 1.0 when walking, 0.0 when standing
    // Using velocity magnitude as heuristic
    bool hasVelocity = std::abs(currentLinVelCmd_[0]) > 0.05 ||
                       std::abs(currentLinVelCmd_[1]) > 0.05 ||
                       std::abs(currentAngVelCmd_) > 0.05;
    currentStandCmd_ = hasVelocity ? 1.0 : currentStandCmd_;

    // Base height command (can be modified externally if needed)
    // For now, use default
    currentBaseHeightCmd_ = defaultBaseHeight_;

    // === Build full observation (500 dims) ===
    int idx = 0;

    // ==== CURRENT OBSERVATION (79 dims) ====

    // 1. last_policy_action (12 dims, lower body)
    observation_.segment(idx, ASAP_LOWER_BODY_JOINTS) = lastPolicyAction_;
    idx += ASAP_LOWER_BODY_JOINTS;

    // 2. base_ang_vel * 0.25 (3 dims)
    observation_.segment<3>(idx) = currentAngVel_ * angVelScale_;
    idx += 3;

    // 3. ang_vel_command (1 dim)
    observation_[idx] = currentAngVelCmd_;
    idx += 1;

    // 4. base_height_command * 2.0 (1 dim)
    observation_[idx] = currentBaseHeightCmd_ * baseHeightScale_;
    idx += 1;

    // 5. lin_vel_command (2 dims)
    observation_.segment<2>(idx) = currentLinVelCmd_;
    idx += 2;

    // 6. stand_command (1 dim)
    observation_[idx] = currentStandCmd_;
    idx += 1;

    // 7. cos_phase (1 dim)
    observation_[idx] = currentCosPhase_;
    idx += 1;

    // 8. dof_pos - default (29 dims)
    observation_.segment(idx, ASAP_NUM_JOINTS) = currentDofPos_;
    idx += ASAP_NUM_JOINTS;

    // 9. dof_vel * 0.05 (29 dims)
    observation_.segment(idx, ASAP_NUM_JOINTS) = currentDofVel_ * dofVelScale_;
    idx += ASAP_NUM_JOINTS;

    // ==== HISTORY (400 dims, ALPHABETICALLY sorted) ====

    // 1. actions (12 * 4 = 48)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_LOWER_BODY_JOINTS, ASAP_LOWER_BODY_JOINTS) = historyActions_[t];
    }
    idx += ASAP_HISTORY_LENGTH * ASAP_LOWER_BODY_JOINTS;

    // 2. base_ang_vel (3 * 4 = 12, already scaled)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyAngVel_[t];
    }
    idx += ASAP_HISTORY_LENGTH * 3;

    // 3. command_ang_vel (1 * 4 = 4)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyAngVelCmd_[t];
    }
    idx += ASAP_HISTORY_LENGTH;

    // 4. command_base_height (1 * 4 = 4, already scaled)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyBaseHeightCmd_[t];
    }
    idx += ASAP_HISTORY_LENGTH;

    // 5. command_lin_vel (2 * 4 = 8)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment<2>(idx + t * 2) = historyLinVelCmd_[t];
    }
    idx += ASAP_HISTORY_LENGTH * 2;

    // 6. command_stand (1 * 4 = 4)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyStandCmd_[t];
    }
    idx += ASAP_HISTORY_LENGTH;

    // 7. cos_phase (1 * 4 = 4)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historyCosPhase_[t];
    }
    idx += ASAP_HISTORY_LENGTH;

    // 8. dof_pos (29 * 4 = 116, already scaled)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_NUM_JOINTS, ASAP_NUM_JOINTS) = historyDofPos_[t];
    }
    idx += ASAP_HISTORY_LENGTH * ASAP_NUM_JOINTS;

    // 9. dof_vel (29 * 4 = 116, already scaled)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_NUM_JOINTS, ASAP_NUM_JOINTS) = historyDofVel_[t];
    }
    idx += ASAP_HISTORY_LENGTH * ASAP_NUM_JOINTS;

    // 10. projected_gravity (3 * 4 = 12)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment<3>(idx + t * 3) = historyGravity_[t];
    }
    idx += ASAP_HISTORY_LENGTH * 3;

    // 11. ref_upper_dof_pos (17 * 4 = 68)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_.segment(idx + t * ASAP_UPPER_BODY_JOINTS, ASAP_UPPER_BODY_JOINTS) = historyRefUpperDofPos_[t];
    }
    idx += ASAP_HISTORY_LENGTH * ASAP_UPPER_BODY_JOINTS;

    // 12. sin_phase (1 * 4 = 4)
    for (int t = 0; t < ASAP_HISTORY_LENGTH; ++t) {
        observation_[idx + t] = historySinPhase_[t];
    }
    idx += ASAP_HISTORY_LENGTH;

    // ==== POST-HISTORY (21 dims) ====

    // 1. projected_gravity (3 dims)
    observation_.segment<3>(idx) = currentGravity_;
    idx += 3;

    // 2. ref_upper_dof_pos (17 dims)
    observation_.segment(idx, ASAP_UPPER_BODY_JOINTS) = refUpperDofPos_;
    idx += ASAP_UPPER_BODY_JOINTS;

    // 3. sin_phase (1 dim)
    observation_[idx] = currentSinPhase_;
    idx += 1;

    // Sanity check
    if (idx != ASAP_TOTAL_OBS_SIZE) {
        TBAI_LOG_ERROR(logger_, "Observation size mismatch: {} != {}", idx, ASAP_TOTAL_OBS_SIZE);
    }

    // Clip observation
    const scalar_t clipObs = 100.0;
    for (int i = 0; i < ASAP_TOTAL_OBS_SIZE; ++i) {
        observation_[i] = std::clamp(observation_[i], -clipObs, clipObs);
    }
}

void G1ASAPController::updateHistory() {
    // Push current observations to history (newest first - position 0 is most recent)
    // This matches ASAP's HistoryHandler which stores newest at index 0

    // actions (with scaling)
    historyActions_.pop_back();
    historyActions_.push_front(lastPolicyAction_);

    // base_ang_vel (with scaling)
    historyAngVel_.pop_back();
    historyAngVel_.push_front(currentAngVel_ * angVelScale_);

    // command_ang_vel
    historyAngVelCmd_.pop_back();
    historyAngVelCmd_.push_front(currentAngVelCmd_);

    // command_base_height (with scaling)
    historyBaseHeightCmd_.pop_back();
    historyBaseHeightCmd_.push_front(currentBaseHeightCmd_ * baseHeightScale_);

    // command_lin_vel
    historyLinVelCmd_.pop_back();
    historyLinVelCmd_.push_front(currentLinVelCmd_);

    // command_stand
    historyStandCmd_.pop_back();
    historyStandCmd_.push_front(currentStandCmd_);

    // cos_phase
    historyCosPhase_.pop_back();
    historyCosPhase_.push_front(currentCosPhase_);

    // dof_pos
    historyDofPos_.pop_back();
    historyDofPos_.push_front(currentDofPos_);

    // dof_vel (with scaling)
    historyDofVel_.pop_back();
    historyDofVel_.push_front(currentDofVel_ * dofVelScale_);

    // projected_gravity
    historyGravity_.pop_back();
    historyGravity_.push_front(currentGravity_);

    // ref_upper_dof_pos
    historyRefUpperDofPos_.pop_back();
    historyRefUpperDofPos_.push_front(refUpperDofPos_);

    // sin_phase
    historySinPhase_.pop_back();
    historySinPhase_.push_front(currentSinPhase_);
}

void G1ASAPController::runInference() {
    if (!modelLoaded_) {
        TBAI_LOG_ERROR(logger_, "Model not loaded!");
        return;
    }

    try {
        // Prepare input tensor
        std::vector<int64_t> inputShape = {1, ASAP_TOTAL_OBS_SIZE};
        std::vector<float> inputData(observation_.data(), observation_.data() + observation_.size());

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor =
            Ort::Value::CreateTensor<float>(memoryInfo, inputData.data(), inputData.size(), inputShape.data(), 2);

        // Run inference
        const char *inputNames[] = {inputName_.c_str()};
        const char *outputNames[] = {outputName_.c_str()};

        auto outputTensors = ortSession_->Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);

        // Get output (12 lower body actions)
        float *outputData = outputTensors[0].GetTensorMutableData<float>();

        // Store raw action for next step's observation (BEFORE clipping/scaling)
        for (int i = 0; i < ASAP_LOWER_BODY_JOINTS; ++i) {
            lastPolicyAction_[i] = static_cast<scalar_t>(outputData[i]);
        }

        // Apply clipping and scaling for motor commands
        // Lower body (first 12 joints)
        for (int i = 0; i < ASAP_LOWER_BODY_JOINTS; ++i) {
            scalar_t rawAction = lastPolicyAction_[i];
            action_[i] = std::clamp(rawAction, -actionClip_, actionClip_) * actionScale_;
        }

        // Upper body (last 17 joints) - use reference directly
        for (int i = 0; i < ASAP_UPPER_BODY_JOINTS; ++i) {
            action_[ASAP_LOWER_BODY_JOINTS + i] = refUpperDofPos_[i];
        }

    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Inference failed: {}", e.what());
    }
}

std::vector<tbai::MotorCommand> G1ASAPController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();
    updateHistory();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    // Create commands for all 29 G1 joints
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        cmd.joint_name = jointNames_[i];

        if (i < ASAP_LOWER_BODY_JOINTS) {
            // Lower body: policy output + default
            cmd.desired_position = action_[i] + defaultJointPos_[i];
        } else {
            // Upper body: reference position (action_[i] already contains ref for upper body)
            // Upper body reference is relative to default, so add default
            cmd.desired_position = action_[i] + defaultJointPos_[i];
        }

        cmd.desired_velocity = 0.0;
        cmd.kp = stiffness_[i];
        cmd.kd = damping_[i];
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    return commands;
}

bool G1ASAPController::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1ASAPController" ||
           controllerType == "G1ASAPLocomotion" || controllerType == "g1_asap" || controllerType == "asap";
}

void G1ASAPController::stopController() {
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1ASAPController::ok() const {
    return true;
}

bool G1ASAPController::checkStability() const {
    const scalar_t maxAngle = 1.0;  // ~57 degrees
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

void G1ASAPController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset actions
    lastPolicyAction_.setZero();
    action_.setZero();

    // Reset phase
    phaseTime_ = 0.0;
    currentSinPhase_ = 0.0;
    currentCosPhase_ = 1.0;

    // Reset stand command
    currentStandCmd_ = 0.0;

    // Clear history buffers
    for (int i = 0; i < ASAP_HISTORY_LENGTH; ++i) {
        historyActions_[i].setZero();
        historyAngVel_[i].setZero();
        historyAngVelCmd_[i] = 0.0;
        historyBaseHeightCmd_[i] = defaultBaseHeight_ * baseHeightScale_;
        historyLinVelCmd_[i].setZero();
        historyStandCmd_[i] = 0.0;
        historyCosPhase_[i] = 1.0;
        historyDofPos_[i].setZero();
        historyDofVel_[i].setZero();
        historyGravity_[i].setZero();
        historyRefUpperDofPos_[i] = refUpperDofPos_;
        historySinPhase_[i] = 0.0;
    }

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    TBAI_LOG_INFO(logger_, "{} ready", controllerName_);
}

}  // namespace g1
}  // namespace tbai
