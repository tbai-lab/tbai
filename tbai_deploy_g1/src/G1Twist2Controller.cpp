#include "tbai_deploy_g1/G1Twist2Controller.hpp"

#include <algorithm>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>

namespace tbai {
namespace g1 {

G1Twist2Controller::G1Twist2Controller(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                       const std::string &policyPath, const std::string &motionFilePath,
                                       float timeStart, float timeEnd, const std::string &controllerName)
    : stateSubscriberPtr_(stateSubscriberPtr),
      modelLoaded_(false),
      timeStart_(timeStart),
      currentMotionTime_(0.0f),
      motionActive_(false),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {} (TWIST2)", controllerName_);

    // Load configuration
    rate_ = tbai::fromGlobalConfig<float>("g1_twist_controller/rate", 50.0f);
    actionScale_ = tbai::fromGlobalConfig<float>("g1_twist_controller/action_scale", 0.5f);
    actionClip_ = tbai::fromGlobalConfig<float>("g1_twist_controller/action_clip", 10.0f);

    angVelScale_ = tbai::fromGlobalConfig<float>("g1_twist_controller/ang_vel_scale", 0.25f);
    dofPosScale_ = tbai::fromGlobalConfig<float>("g1_twist_controller/dof_pos_scale", 1.0f);
    dofVelScale_ = tbai::fromGlobalConfig<float>("g1_twist_controller/dof_vel_scale", 0.05f);

    // Load ankle joint indices for velocity zeroing
    ankleJointIds_ = tbai::fromGlobalConfig<std::vector<int>>("g1_twist_controller/ankle_joint_ids");

    // Load default joint positions (29 G1 joints)
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_twist_controller/default_joint_pos");
    if (defaultJointPosVec.size() != TWIST2_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 default_joint_pos values");
    }
    defaultJointPos_ = vector_t(TWIST2_NUM_JOINTS);
    for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
    }

    // Load stiffness and damping (29 joints)
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_twist_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_twist_controller/damping");
    if (stiffnessVec.size() != TWIST2_NUM_JOINTS || dampingVec.size() != TWIST2_NUM_JOINTS) {
        throw std::runtime_error("Expected 29 stiffness/damping values");
    }
    stiffness_ = vector_t(TWIST2_NUM_JOINTS);
    damping_ = vector_t(TWIST2_NUM_JOINTS);
    for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load motion data
    motionLoader_ = std::make_unique<Twist2MotionLoader>(motionFilePath);
    TBAI_LOG_INFO(logger_, "Loaded motion file with {} frames, duration: {:.2f}s", motionLoader_->getNumFrames(),
                  motionLoader_->getDuration());

    // Set time end (use full duration if -1)
    timeEnd_ = (timeEnd < 0) ? motionLoader_->getDuration() : timeEnd;
    timeEnd_ = std::clamp(timeEnd_, 0.0f, motionLoader_->getDuration());
    timeStart_ = std::clamp(timeStart_, 0.0f, timeEnd_);

    motionLoader_->setTimeRange(timeStart_, timeEnd_);

    TBAI_LOG_INFO(logger_, "Motion playback range: {:.2f}s - {:.2f}s", timeStart_, timeEnd_);

    // Initialize vectors
    lastAction_ = vector_t::Zero(TWIST2_NUM_JOINTS);
    action_ = vector_t::Zero(TWIST2_NUM_JOINTS);
    observation_ = vector_t::Zero(TWIST2_TOTAL_OBS_SIZE);
    currentFrameObs_ = vector_t::Zero(TWIST2_OBS_FRAME_SIZE);
    actionMimicObs_ = vector_t::Zero(TWIST2_OBS_ACTION_MIMIC);
    proprioObs_ = vector_t::Zero(TWIST2_OBS_PROPRIO);

    // Initialize history buffer with zeros
    for (int i = 0; i < TWIST2_HISTORY_LENGTH; ++i) {
        historyBuffer_.push_back(vector_t::Zero(TWIST2_OBS_FRAME_SIZE));
    }

    // Initialize ONNX Runtime model
    initOnnxModel(policyPath);

    TBAI_LOG_INFO(logger_, "{} initialized successfully (obs_size={})", controllerName_, TWIST2_TOTAL_OBS_SIZE);
}

G1Twist2Controller::~G1Twist2Controller() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
}

void G1Twist2Controller::initOnnxModel(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    try {
        // Create ONNX Runtime environment
        ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1Twist2Controller");

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

void G1Twist2Controller::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();

    // Update motion time
    if (motionActive_) {
        motionLoader_->advance(static_cast<float>(dt));
        currentMotionTime_ = motionLoader_->getCurrentTime() - timeStart_;
    }
}

vector_t G1Twist2Controller::buildActionMimicObs() const {
    // Get action_mimic (35 dims) from motion loader
    // Format: xy_vel(2), z(1), roll(1), pitch(1), yaw_ang_vel(1), dof_pos(29) = 35
    return motionLoader_->getActionMimic();
}

Eigen::Vector2d G1Twist2Controller::quatToRollPitch(const vector4_t &quat) const {
    // Convert quaternion [x, y, z, w] to roll/pitch using TWIST2's formula
    // This matches TWIST2's quatToEuler function in rot_utils.py
    double x = quat(0);
    double y = quat(1);
    double z = quat(2);
    double w = quat(3);

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    return Eigen::Vector2d(roll, pitch);
}

vector_t G1Twist2Controller::buildProprioObs() const {
    // Build proprio observation (92 dims):
    // ang_vel(3)*scale + roll_pitch(2) + (dof_pos-default)(29)*scale + dof_vel(29)*scale + last_action(29)

    vector_t proprio(TWIST2_OBS_PROPRIO);
    int idx = 0;

    // Get state components
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Zero ankle velocities
    vector_t jointVelZeroed = jointVel;
    for (int ankleIdx : ankleJointIds_) {
        jointVelZeroed[ankleIdx] = 0.0;
    }

    // 1. Angular velocity (3 dims, scaled)
    proprio.segment<3>(idx) = baseAngVel * angVelScale_;
    idx += 3;

    // 2. Roll & Pitch (2 dims) - compute from raw quaternion using TWIST2's formula
    // Try to get raw quaternion from G1RobotInterface for exact TWIST2 compatibility
    Eigen::Vector2d rollPitch;
    auto *g1Interface = dynamic_cast<G1RobotInterface *>(stateSubscriberPtr_.get());
    if (g1Interface) {
        vector4_t quat = g1Interface->getBaseQuaternion();
        rollPitch = quatToRollPitch(quat);
    } else {
        // Fallback to OCS2 RPY if not G1RobotInterface
        rollPitch(0) = state_.x(0);  // roll
        rollPitch(1) = state_.x(1);  // pitch
    }
    proprio[idx] = rollPitch(0);      // roll
    proprio[idx + 1] = rollPitch(1);  // pitch
    idx += 2;

    // 3. DOF positions - default (29 dims, scaled)
    for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
        proprio[idx + i] = (jointPos[i] - defaultJointPos_[i]) * dofPosScale_;
    }
    idx += TWIST2_NUM_JOINTS;

    // 4. DOF velocities (29 dims, scaled, ankles zeroed)
    for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
        proprio[idx + i] = jointVelZeroed[i] * dofVelScale_;
    }
    idx += TWIST2_NUM_JOINTS;

    // 5. Last action (29 dims)
    proprio.segment(idx, TWIST2_NUM_JOINTS) = lastAction_;

    return proprio;
}

void G1Twist2Controller::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Get action_mimic and proprio
    actionMimicObs_ = buildActionMimicObs();
    proprioObs_ = buildProprioObs();

    // === Build current frame observation (127 dims) ===
    // Structure: action_mimic(35) + proprio(92) = 127
    currentFrameObs_.segment(0, TWIST2_OBS_ACTION_MIMIC) = actionMimicObs_;
    currentFrameObs_.segment(TWIST2_OBS_ACTION_MIMIC, TWIST2_OBS_PROPRIO) = proprioObs_;

    // === Build full observation: current(127) + history(10 * 127) + future_mimic(35) = 1432 dims ===
    int idx = 0;

    // Current frame (127 dims)
    observation_.segment(idx, TWIST2_OBS_FRAME_SIZE) = currentFrameObs_;
    idx += TWIST2_OBS_FRAME_SIZE;

    // History (10 * 127 = 1270 dims)
    observation_.segment(idx, TWIST2_HISTORY_LENGTH * TWIST2_OBS_FRAME_SIZE) = buildHistoryObs();
    idx += TWIST2_HISTORY_LENGTH * TWIST2_OBS_FRAME_SIZE;

    // Future action_mimic (35 dims) - same as current for now
    observation_.segment(idx, TWIST2_OBS_ACTION_MIMIC) = actionMimicObs_;
}

vector_t G1Twist2Controller::buildHistoryObs() const {
    vector_t historyObs(TWIST2_HISTORY_LENGTH * TWIST2_OBS_FRAME_SIZE);

    for (int i = 0; i < TWIST2_HISTORY_LENGTH; ++i) {
        historyObs.segment(i * TWIST2_OBS_FRAME_SIZE, TWIST2_OBS_FRAME_SIZE) = historyBuffer_[i];
    }

    return historyObs;
}

void G1Twist2Controller::updateHistoryBuffer() {
    // Push current frame to history (FIFO)
    historyBuffer_.pop_front();
    historyBuffer_.push_back(currentFrameObs_);
}

void G1Twist2Controller::runInference() {
    if (!modelLoaded_) {
        TBAI_LOG_ERROR(logger_, "Model not loaded!");
        return;
    }

    try {
        // Prepare input tensor
        std::vector<int64_t> inputShape = {1, TWIST2_TOTAL_OBS_SIZE};
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
        for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
            lastAction_[i] = static_cast<scalar_t>(outputData[i]);
        }

        // Apply clipping and scaling for motor commands
        for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
            scalar_t rawAction = lastAction_[i];
            action_[i] = std::clamp(rawAction, -actionClip_, actionClip_) * actionScale_;
        }

    } catch (const Ort::Exception &e) {
        TBAI_LOG_ERROR(logger_, "Inference failed: {}", e.what());
    }
}

std::vector<tbai::MotorCommand> G1Twist2Controller::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();
    updateHistoryBuffer();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    // Create commands for all 29 joints
    for (int i = 0; i < TWIST2_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        cmd.joint_name = jointNames_[i];
        cmd.desired_position = action_[i] + defaultJointPos_[i];
        cmd.desired_velocity = 0.0;
        cmd.kp = stiffness_[i];
        cmd.kd = damping_[i];
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    return commands;
}

bool G1Twist2Controller::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1Twist2Controller" ||
           controllerType == "g1_twist" || controllerType == "twist" || controllerType == "twist2";
}

void G1Twist2Controller::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1Twist2Controller::ok() const {
    return true;
}

bool G1Twist2Controller::checkStability() const {
    const scalar_t maxAngle = 1.0;  // ~57 degrees
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

bool G1Twist2Controller::isMotionComplete() const {
    return motionLoader_->isComplete();
}

void G1Twist2Controller::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset actions
    lastAction_.setZero();
    action_.setZero();

    // Clear history buffer
    for (auto &hist : historyBuffer_) {
        hist.setZero();
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
