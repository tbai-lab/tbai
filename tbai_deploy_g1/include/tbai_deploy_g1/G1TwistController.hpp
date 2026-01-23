#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_deploy_g1/G1Constants.hpp>
#include <tbai_deploy_g1/Twist2MotionLoader.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

namespace tbai {
namespace g1 {

// TWIST2 Controller Constants (29 DOFs, ONNX)
constexpr int TWIST2_NUM_JOINTS = 29;      // TWIST2 uses all 29 DOFs including wrists
constexpr int TWIST2_HISTORY_LENGTH = 10;  // 10 frames of history

// Action mimic from motion: xy_vel(2) + z(1) + roll_pitch(2) + yaw_ang_vel(1) + dof_pos(29) = 35
constexpr int TWIST2_OBS_ACTION_MIMIC = 35;

// Proprio observation: ang_vel(3) + roll_pitch(2) + dof_pos(29) + dof_vel(29) + last_action(29) = 92
constexpr int TWIST2_OBS_PROPRIO = 92;

// Observation per frame: action_mimic(35) + proprio(92) = 127
constexpr int TWIST2_OBS_FRAME_SIZE = 127;

// Total observation: current(127) + history(10 * 127) + future_mimic(35) = 1432
constexpr int TWIST2_TOTAL_OBS_SIZE = TWIST2_OBS_FRAME_SIZE * (TWIST2_HISTORY_LENGTH + 1) + TWIST2_OBS_ACTION_MIMIC;

/**
 * @brief G1 TWIST2 Controller for motion tracking using ONNX Runtime.
 *
 * This controller implements the TWIST2 motion tracking algorithm from amazon-far/TWIST2.
 * It uses a neural network policy to track motion commands (action_mimic) from
 * motion file playback and outputs joint position targets for all 29 DOFs.
 *
 * Key features:
 * - 29 DOF control (all joints including wrists)
 * - 1432-dim observation with 10-frame history
 * - ONNX Runtime model inference
 * - Motion from TWIST2 pkl format
 */
class G1TwistController : public tbai::Controller {
   public:
    /**
     * @brief Construct a new G1TwistController (TWIST2)
     * @param stateSubscriberPtr State subscriber for robot state
     * @param policyPath Path to ONNX model (.onnx file)
     * @param motionFilePath Path to motion pkl file
     * @param timeStart Start time in motion file
     * @param timeEnd End time in motion file (-1 for full duration)
     * @param controllerName Name for logging
     */
    G1TwistController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr, const std::string &policyPath,
                      const std::string &motionFilePath, float timeStart = 0.0f, float timeEnd = -1.0f,
                      const std::string &controllerName = "G1TwistController");

    ~G1TwistController();

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return controllerName_; }

    void stopController() override;

    bool ok() const override;

    scalar_t getRate() const override { return rate_; }

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isMotionComplete() const;

    float getMotionTime() const { return currentMotionTime_; }

   protected:
    void initOnnxModel(const std::string &policyPath);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();
    void updateHistoryBuffer();

    /**
     * @brief Build the action_mimic observation (35 dims)
     * From motion: xy_vel(2), z(1), roll_pitch(2), yaw_ang_vel(1), dof_pos(29)
     */
    vector_t buildActionMimicObs() const;

    /**
     * @brief Build proprio observation (92 dims)
     * ang_vel(3)*scale + roll_pitch(2) + (dof_pos-default)(29)*scale + dof_vel(29)*scale + last_action(29)
     */
    vector_t buildProprioObs() const;

    /**
     * @brief Build history observation vector from buffer
     */
    vector_t buildHistoryObs() const;

    /**
     * @brief Compute roll and pitch from quaternion using TWIST2's formula
     * This ensures exact compatibility with TWIST2's euler angle computation.
     * @param quat Quaternion in [x, y, z, w] format
     * @return Vector2 containing [roll, pitch]
     */
    Eigen::Vector2d quatToRollPitch(const vector4_t& quat) const;

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::unique_ptr<Twist2MotionLoader> motionLoader_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::SessionOptions> ortSessionOptions_;
    std::string inputName_;
    std::string outputName_;
    bool modelLoaded_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    // Observation components
    vector_t observation_;       // Full 1432-dim observation
    vector_t currentFrameObs_;   // Current frame (127 dims)
    vector_t actionMimicObs_;    // Current action_mimic (35 dims)
    vector_t proprioObs_;        // Current proprio (92 dims)

    // History buffer (deque of observation frames, 127 dims each)
    std::deque<vector_t> historyBuffer_;

    // Action output
    vector_t action_;

    // Configuration (29 DOFs)
    vector_t defaultJointPos_;   // 29 G1 joints
    vector_t stiffness_;         // 29 joints
    vector_t damping_;           // 29 joints
    std::vector<int> ankleJointIds_;  // Ankle joint indices for velocity zeroing

    std::vector<std::string> jointNames_;  // 29 joint names

    // Observation scales
    scalar_t angVelScale_;
    scalar_t dofPosScale_;
    scalar_t dofVelScale_;

    // Action processing
    scalar_t actionScale_;
    scalar_t actionClip_;

    // Motion playback
    float timeStart_;
    float timeEnd_;
    float currentMotionTime_;
    bool motionActive_;

    scalar_t rate_;

    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
