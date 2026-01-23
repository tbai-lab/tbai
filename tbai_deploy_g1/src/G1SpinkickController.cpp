#include "tbai_deploy_g1/G1SpinkickController.hpp"

#include <algorithm>
#include <cstring>
#include <sstream>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace g1 {

// Observation sizes for Spinkick (same structure as BeyondMimic)
constexpr int SPINKICK_OBS_MOTION_CMD = 58;       // 29 joint_pos + 29 joint_vel
constexpr int SPINKICK_OBS_ANCHOR_ORI = 6;
constexpr int SPINKICK_OBS_BASE_ANG_VEL = 3;
constexpr int SPINKICK_OBS_JOINT_POS_REL = 29;
constexpr int SPINKICK_OBS_JOINT_VEL_REL = 29;
constexpr int SPINKICK_OBS_LAST_ACTION = 29;
constexpr int SPINKICK_TOTAL_OBS_SIZE = SPINKICK_OBS_MOTION_CMD + SPINKICK_OBS_ANCHOR_ORI + SPINKICK_OBS_BASE_ANG_VEL +
                                        SPINKICK_OBS_JOINT_POS_REL + SPINKICK_OBS_JOINT_VEL_REL + SPINKICK_OBS_LAST_ACTION;

G1SpinkickController::G1SpinkickController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                           const std::string &policyPath, const std::string &controllerName,
                                           bool useModelMetaConfig, float actionBeta)
    : stateSubscriberPtr_(stateSubscriberPtr),
      actionBeta_(actionBeta),
      useModelMetaConfig_(useModelMetaConfig),
      timestep_(0),
      maxTimestep_(-1),
      motionActive_(false),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {}", controllerName_);

    // Initialize vectors
    lastAction_ = vector_t::Zero(G1_NUM_JOINTS);
    action_ = vector_t::Zero(G1_NUM_JOINTS);
    rawAction_ = vector_t::Zero(G1_NUM_JOINTS);
    observation_ = vector_t::Zero(SPINKICK_TOTAL_OBS_SIZE);

    // Initialize motion data
    motionJointPos_ = vector_t::Zero(G1_NUM_JOINTS);
    motionJointVel_ = vector_t::Zero(G1_NUM_JOINTS);

    // Load configuration
    defaultJointPos_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/default_joint_pos");
    actionScale_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/action_scale");
    stiffness_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/stiffness");
    damping_ = tbai::fromGlobalConfig<vector_t>("g1_spinkick_controller/damping");

    // Initialize ONNX Runtime first to potentially get metadata
    initOnnxRuntime(policyPath);

    // Parse model metadata if available and enabled
    if (useModelMetaConfig_) {
        parseModelMetadata();
    }

    jointIdsMap_ = tbai::fromGlobalConfig<std::vector<int>>("g1_spinkick_controller/joint_ids_map");
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    maxTimestep_ = tbai::fromGlobalConfig<int>("g1_spinkick_controller/max_timestep", -1);
    actionBeta_ = tbai::fromGlobalConfig<float>("g1_spinkick_controller/action_beta", actionBeta);

    TBAI_LOG_INFO(logger_, "{} initialized successfully (actionBeta={:.2f}, maxTimestep={})", controllerName_,
                  actionBeta_, maxTimestep_);
}

G1SpinkickController::~G1SpinkickController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
    for (auto name : inputNames_) {
        free(const_cast<char *>(name));
    }
    for (auto name : outputNames_) {
        free(const_cast<char *>(name));
    }
}

void G1SpinkickController::initOnnxRuntime(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1SpinkickController");

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), sessionOptions);
    memoryInfo_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Get input names
    Ort::AllocatorWithDefaultOptions allocator;
    size_t numInputs = ortSession_->GetInputCount();
    TBAI_LOG_INFO(logger_, "Model has {} inputs", numInputs);

    for (size_t i = 0; i < numInputs; ++i) {
        auto inputNamePtr = ortSession_->GetInputNameAllocated(i, allocator);
        inputNames_.push_back(strdup(inputNamePtr.get()));
        TBAI_LOG_INFO(logger_, "  Input {}: {}", i, inputNames_.back());
    }

    // Get output names
    size_t numOutputs = ortSession_->GetOutputCount();
    TBAI_LOG_INFO(logger_, "Model has {} outputs", numOutputs);

    for (size_t i = 0; i < numOutputs; ++i) {
        auto outputNamePtr = ortSession_->GetOutputNameAllocated(i, allocator);
        outputNames_.push_back(strdup(outputNamePtr.get()));
        TBAI_LOG_INFO(logger_, "  Output {}: {}", i, outputNames_.back());
    }
}

void G1SpinkickController::parseModelMetadata() {
    TBAI_LOG_INFO(logger_, "Parsing model metadata...");

    try {
        Ort::AllocatorWithDefaultOptions allocator;
        auto metadata = ortSession_->GetModelMetadata();

        auto parseFloats = [](const std::string &str) {
            std::vector<double> result;
            std::stringstream ss(str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                result.push_back(std::stod(token));
            }
            return result;
        };

        bool hasMetadata = false;
        std::vector<std::string> metaKeys = {"default_joint_pos", "joint_stiffness", "joint_damping", "action_scale"};

        for (const auto &key : metaKeys) {
            try {
                auto valuePtr = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
                if (valuePtr) {
                    hasMetadata = true;
                    std::string value = valuePtr.get();

                    if (key == "default_joint_pos") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            defaultJointPos_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded default_joint_pos from metadata ({} values)", values.size());
                    } else if (key == "joint_stiffness") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            stiffness_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded stiffness from metadata ({} values)", values.size());
                    } else if (key == "joint_damping") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            damping_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded damping from metadata ({} values)", values.size());
                    } else if (key == "action_scale") {
                        auto values = parseFloats(value);
                        for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(G1_NUM_JOINTS)); ++i) {
                            actionScale_[i] = values[i];
                        }
                        TBAI_LOG_INFO(logger_, "Loaded action_scale from metadata ({} values)", values.size());
                    }
                }
            } catch (...) {
                TBAI_LOG_ERROR(logger_, "Failed to parse {} from model metadata!", key);
            }
        }

        if (!hasMetadata) {
            TBAI_LOG_WARN(logger_, "No custom metadata found in model");
        }

    } catch (const std::exception &e) {
        TBAI_LOG_WARN(logger_, "Failed to parse model metadata: {}", e.what());
    }
}

void G1SpinkickController::preStep(scalar_t currentTime, scalar_t dt) {
    state_ = stateSubscriberPtr_->getLatestState();
}

void G1SpinkickController::postStep(scalar_t currentTime, scalar_t dt) {
    if (motionActive_) {
        timestep_++;
        if (maxTimestep_ > 0 && timestep_ >= maxTimestep_) {
            TBAI_LOG_INFO(logger_, "Motion complete (reached max timestep {})", maxTimestep_);
            motionActive_ = false;
        }
    }
}

void G1SpinkickController::buildObservation(scalar_t currentTime, scalar_t dt) {
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    int idx = 0;

    // Motion command (from model outputs of previous step)
    observation_.segment(idx, G1_NUM_JOINTS) = motionJointPos_;
    idx += G1_NUM_JOINTS;
    observation_.segment(idx, G1_NUM_JOINTS) = motionJointVel_;
    idx += G1_NUM_JOINTS;

    // Anchor orientation (simplified - identity for now)
    observation_.segment<6>(idx).setZero();
    idx += 6;

    // Base angular velocity
    observation_.segment<3>(idx) = baseAngVel;
    idx += 3;

    // Joint positions relative to default
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int ddsIdx = jointIdsMap_[i];
        observation_[idx + i] = jointPos[ddsIdx] - defaultJointPos_[i];
    }
    idx += G1_NUM_JOINTS;

    // Joint velocities
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int ddsIdx = jointIdsMap_[i];
        observation_[idx + i] = jointVel[ddsIdx];
    }
    idx += G1_NUM_JOINTS;

    // Last action
    observation_.segment(idx, G1_NUM_JOINTS) = lastAction_;
}

void G1SpinkickController::runInference() {
    std::vector<int64_t> obsShape = {1, SPINKICK_TOTAL_OBS_SIZE};
    std::vector<float> obsData(SPINKICK_TOTAL_OBS_SIZE);
    for (int i = 0; i < SPINKICK_TOTAL_OBS_SIZE; ++i) {
        obsData[i] = static_cast<float>(observation_[i]);
    }

    Ort::Value obsTensor =
        Ort::Value::CreateTensor<float>(*memoryInfo_, obsData.data(), obsData.size(), obsShape.data(), obsShape.size());

    std::vector<int64_t> timestepShape = {1, 1};
    std::vector<float> timestepData = {static_cast<float>(timestep_)};

    Ort::Value timestepTensor = Ort::Value::CreateTensor<float>(*memoryInfo_, timestepData.data(), timestepData.size(),
                                                                timestepShape.data(), timestepShape.size());

    std::vector<Ort::Value> inputTensors;
    inputTensors.push_back(std::move(obsTensor));
    inputTensors.push_back(std::move(timestepTensor));

    auto outputTensors = ortSession_->Run(Ort::RunOptions{nullptr}, inputNames_.data(), inputTensors.data(),
                                          inputTensors.size(), outputNames_.data(), outputNames_.size());

    // Extract actions
    float *actionsData = outputTensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        rawAction_[i] = static_cast<scalar_t>(actionsData[i]);
    }

    // Apply action smoothing
    action_ = (1.0 - actionBeta_) * lastAction_ + actionBeta_ * rawAction_;
    lastAction_ = action_;

    // Get motion data from model outputs if available
    if (outputTensors.size() > 1) {
        float *jointPosData = outputTensors[1].GetTensorMutableData<float>();
        for (int i = 0; i < G1_NUM_JOINTS; ++i) {
            motionJointPos_[i] = static_cast<scalar_t>(jointPosData[i]);
        }

        if (outputTensors.size() > 2) {
            float *jointVelData = outputTensors[2].GetTensorMutableData<float>();
            for (int i = 0; i < G1_NUM_JOINTS; ++i) {
                motionJointVel_[i] = static_cast<scalar_t>(jointVelData[i]);
            }
        }
    }
}

std::vector<tbai::MotorCommand> G1SpinkickController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;
        int ddsIdx = jointIdsMap_[i];
        cmd.joint_name = jointNames_[ddsIdx];
        cmd.desired_position = actionScale_[i] * action_[i] + defaultJointPos_[i];
        cmd.desired_velocity = 0.0;
        cmd.kp = stiffness_[i];
        cmd.kd = damping_[i];
        cmd.torque_ff = 0.0;
        commands.push_back(cmd);
    }

    return commands;
}

bool G1SpinkickController::isSupported(const std::string &controllerType) {
    return controllerType == controllerName_ || controllerType == "G1SpinkickController" ||
           controllerType == "g1_spinkick" || controllerType == "spinkick";
}

void G1SpinkickController::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1SpinkickController::ok() const {
    return true;
}

bool G1SpinkickController::checkStability() const {
    constexpr scalar_t maxAngle = 1.0;
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

bool G1SpinkickController::isMotionComplete() const {
    return maxTimestep_ > 0 && timestep_ >= maxTimestep_;
}

void G1SpinkickController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset action
    lastAction_.setZero();
    action_.setZero();
    rawAction_.setZero();

    // Reset timestep
    timestep_ = 0;
    motionActive_ = true;

    // Reset motion data
    motionJointPos_.setZero();
    motionJointVel_.setZero();

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    // Run initial inference at timestep 0
    observation_.setZero();
    runInference();

    TBAI_LOG_INFO(logger_, "Motion playback started (timestep=0)");
}

}  // namespace g1
}  // namespace tbai
