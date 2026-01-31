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

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

namespace tbai {
namespace g1 {

// ASAP Mimic Controller Constants (23 DOF with locked wrists)
constexpr int ASAP_MIMIC_NUM_JOINTS = 23;  // 29 - 6 wrist joints
constexpr int ASAP_MIMIC_HISTORY_LENGTH = 4;

// Current observation: actions(23) + base_ang_vel(3) + dof_pos(23) + dof_vel(23) = 72
// History (ALPHABETICALLY sorted): actions(23*4) + base_ang_vel(3*4) + dof_pos(23*4) + dof_vel(23*4) +
//                                   projected_gravity(3*4) + ref_motion_phase(1*4) = 304
// Post-history: projected_gravity(3) + ref_motion_phase(1) = 4
// Total: 72 + 304 + 4 = 380
constexpr int ASAP_MIMIC_CURRENT_OBS_SIZE = 72;
constexpr int ASAP_MIMIC_HISTORY_OBS_SIZE = 304;
constexpr int ASAP_MIMIC_POST_HISTORY_SIZE = 4;
constexpr int ASAP_MIMIC_TOTAL_OBS_SIZE =
    ASAP_MIMIC_CURRENT_OBS_SIZE + ASAP_MIMIC_HISTORY_OBS_SIZE + ASAP_MIMIC_POST_HISTORY_SIZE;

/**
 * @brief G1 ASAP Mimic Controller for motion tracking using ONNX Runtime.
 *
 * This controller implements ASAP motion tracking for specific motions like CR7 celebration.
 * It uses 23 DOFs (locked wrists) and tracks a reference motion phase.
 *
 * Observation structure (380 dims):
 * Current (72 dims):
 * 1. last_action: 23
 * 2. base_ang_vel * 0.25: 3
 * 3. dof_pos - default: 23
 * 4. dof_vel * 0.05: 23
 *
 * History (304 dims, ALPHABETICALLY sorted):
 * 1. actions (23 * 4 = 92)
 * 2. base_ang_vel (3 * 4 = 12)
 * 3. dof_pos (23 * 4 = 92)
 * 4. dof_vel (23 * 4 = 92)
 * 5. projected_gravity (3 * 4 = 12)
 * 6. ref_motion_phase (1 * 4 = 4)
 *
 * Post-History (4 dims):
 * 1. projected_gravity: 3
 * 2. ref_motion_phase: 1
 */
class G1ASAPMimicController : public tbai::Controller {
   public:
    G1ASAPMimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                          const std::string &policyPath, float motionLength,
                          const std::string &controllerName = "G1ASAPMimic");

    ~G1ASAPMimicController();

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

    bool isMotionComplete() const { return motionComplete_; }

    float getMotionPhase() const { return motionPhase_; }

   protected:
    void initOnnxModel(const std::string &policyPath);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();
    void updateHistory();

    /**
     * @brief Get projected gravity vector from quaternion
     */
    vector_t getProjectedGravity(const vector4_t &quat) const;

    /**
     * @brief Map from 23-DOF ASAP mimic order to 29-DOF G1 order
     */
    void initJointMapping();

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::SessionOptions> ortSessionOptions_;
    std::string inputName_;
    std::string outputName_;
    bool modelLoaded_;

    // State
    tbai::State state_;
    vector_t lastAction_;  // 23 dims

    // Full observation (380 dims)
    vector_t observation_;

    // Current observation components (in 23-DOF space)
    vector_t currentAngVel_;   // 3 dims
    vector_t currentGravity_;  // 3 dims
    vector_t currentDofPos_;   // 23 dims (relative to default)
    vector_t currentDofVel_;   // 23 dims
    scalar_t currentPhase_;    // 1 dim (0 to 1)

    // History buffers (ALPHABETICALLY sorted)
    std::deque<vector_t> historyActions_;  // 4 x 23
    std::deque<vector_t> historyAngVel_;   // 4 x 3
    std::deque<vector_t> historyDofPos_;   // 4 x 23
    std::deque<vector_t> historyDofVel_;   // 4 x 23
    std::deque<vector_t> historyGravity_;  // 4 x 3
    std::deque<scalar_t> historyPhase_;    // 4 x 1

    // Action output (23 mimic DOFs)
    vector_t action_;

    // Configuration (23 DOFs for mimic, 29 for full G1)
    vector_t defaultJointPos23_;  // 23 joints (mimic order)
    vector_t defaultJointPos29_;  // 29 joints (G1 order)
    vector_t stiffness_;          // 29 joints
    vector_t damping_;            // 29 joints

    // Joint mapping from 23-DOF mimic to 29-DOF G1
    std::vector<int> mimicToG1Mapping_;  // Maps mimic joint index to G1 joint index
    std::vector<bool> dofMask_;          // Which of 29 DOFs are active (true for 23, false for 6 wrists)

    std::vector<std::string> jointNames_;  // 29 joint names

    // Observation scales
    scalar_t angVelScale_;
    scalar_t dofVelScale_;

    // Action processing
    scalar_t actionScale_;
    scalar_t actionClip_;

    // Motion tracking
    float motionLength_;     // Duration of motion in seconds
    float motionStartTime_;  // When motion started
    float motionPhase_;      // Current phase (0 to 1)
    bool motionActive_;
    bool motionComplete_;

    scalar_t rate_;

    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
