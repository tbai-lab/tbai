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
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

namespace tbai {
namespace g1 {

// ASAP Controller Constants (29 DOF, decoupled control)
constexpr int ASAP_NUM_JOINTS = 29;
constexpr int ASAP_LOWER_BODY_JOINTS = 12;
constexpr int ASAP_UPPER_BODY_JOINTS = 17;
constexpr int ASAP_HISTORY_LENGTH = 4;

// Current observation: last_policy_action(12) + base_ang_vel(3) + ang_vel_cmd(1) + base_height_cmd(1) +
//                      lin_vel_cmd(2) + stand_cmd(1) + cos_phase(1) + dof_pos(29) + dof_vel(29) = 79
constexpr int ASAP_CURRENT_OBS_SIZE = 79;

// History (ALPHABETICALLY sorted):
// actions(12*4=48) + base_ang_vel(3*4=12) + command_ang_vel(1*4=4) + command_base_height(1*4=4) +
// command_lin_vel(2*4=8) + command_stand(1*4=4) + cos_phase(1*4=4) + dof_pos(29*4=116) +
// dof_vel(29*4=116) + projected_gravity(3*4=12) + ref_upper_dof_pos(17*4=68) + sin_phase(1*4=4) = 400
constexpr int ASAP_HISTORY_OBS_SIZE = 400;

// Post-history: projected_gravity(3) + ref_upper_dof_pos(17) + sin_phase(1) = 21
constexpr int ASAP_POST_HISTORY_SIZE = 21;

// Total observation: current(79) + history(400) + post_history(21) = 500
constexpr int ASAP_TOTAL_OBS_SIZE = ASAP_CURRENT_OBS_SIZE + ASAP_HISTORY_OBS_SIZE + ASAP_POST_HISTORY_SIZE;

/**
 * @brief G1 ASAP Controller for decoupled locomotion using ONNX Runtime.
 *
 * This controller implements the ASAP (Agile Skills via Adversarial Play) decoupled
 * locomotion algorithm. It uses a neural network policy that outputs 12 DOFs for the
 * lower body while tracking upper body references for the 17 upper body DOFs.
 *
 * Observation structure (500 dims):
 * Current (79 dims):
 * 1. last_policy_action: 12 (lower body)
 * 2. base_ang_vel * 0.25: 3
 * 3. ang_vel_command: 1
 * 4. base_height_command * 2.0: 1
 * 5. lin_vel_command: 2
 * 6. stand_command: 1
 * 7. cos_phase: 1
 * 8. dof_pos - default: 29
 * 9. dof_vel * 0.05: 29
 *
 * History (400 dims, ALPHABETICALLY sorted):
 * 1. actions (12 * 4 = 48)
 * 2. base_ang_vel (3 * 4 = 12)
 * 3. command_ang_vel (1 * 4 = 4)
 * 4. command_base_height (1 * 4 = 4)
 * 5. command_lin_vel (2 * 4 = 8)
 * 6. command_stand (1 * 4 = 4)
 * 7. cos_phase (1 * 4 = 4)
 * 8. dof_pos (29 * 4 = 116)
 * 9. dof_vel (29 * 4 = 116)
 * 10. projected_gravity (3 * 4 = 12)
 * 11. ref_upper_dof_pos (17 * 4 = 68)
 * 12. sin_phase (1 * 4 = 4)
 *
 * Post-History (21 dims):
 * 1. projected_gravity: 3
 * 2. ref_upper_dof_pos: 17
 * 3. sin_phase: 1
 */
class G1ASAPController : public tbai::Controller {
   public:
    G1ASAPController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                     const std::string &policyPath, const std::string &controllerName = "G1ASAPLocomotion");

    ~G1ASAPController();

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

   protected:
    void initOnnxModel(const std::string &policyPath);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();
    void updateHistory();

    /**
     * @brief Get projected gravity vector from quaternion (ASAP uses [w,x,y,z] format)
     */
    vector_t getProjectedGravity(const vector4_t &quat) const;

    /**
     * @brief Compute phase based on current time and stand command
     */
    void updatePhase(scalar_t dt);

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGenPtr_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::SessionOptions> ortSessionOptions_;
    std::string inputName_;
    std::string outputName_;
    bool modelLoaded_;

    // State
    tbai::State state_;
    vector_t lastPolicyAction_;  // 12 dims (lower body only)

    // Full observation (500 dims)
    vector_t observation_;

    // Current observation components
    vector_t currentAngVel_;       // 3 dims
    vector_t currentGravity_;      // 3 dims
    vector_t currentDofPos_;       // 29 dims (relative to default)
    vector_t currentDofVel_;       // 29 dims
    vector_t currentLinVelCmd_;    // 2 dims
    scalar_t currentAngVelCmd_;    // 1 dim
    scalar_t currentStandCmd_;     // 1 dim
    scalar_t currentBaseHeightCmd_;// 1 dim
    scalar_t currentCosPhase_;     // 1 dim
    scalar_t currentSinPhase_;     // 1 dim

    // Upper body reference
    vector_t refUpperDofPos_;  // 17 dims

    // Phase tracking
    scalar_t phaseTime_;  // Accumulated phase time
    scalar_t gaitPeriod_;

    // History buffers (each is a deque of ASAP_HISTORY_LENGTH elements)
    // CRITICAL: These are stored in alphabetical order for history packing
    std::deque<vector_t> historyActions_;         // 4 x 12
    std::deque<vector_t> historyAngVel_;          // 4 x 3
    std::deque<scalar_t> historyAngVelCmd_;       // 4 x 1
    std::deque<scalar_t> historyBaseHeightCmd_;   // 4 x 1
    std::deque<vector_t> historyLinVelCmd_;       // 4 x 2
    std::deque<scalar_t> historyStandCmd_;        // 4 x 1
    std::deque<scalar_t> historyCosPhase_;        // 4 x 1
    std::deque<vector_t> historyDofPos_;          // 4 x 29
    std::deque<vector_t> historyDofVel_;          // 4 x 29
    std::deque<vector_t> historyGravity_;         // 4 x 3
    std::deque<vector_t> historyRefUpperDofPos_;  // 4 x 17
    std::deque<scalar_t> historySinPhase_;        // 4 x 1

    // Action output (12 lower body + 17 upper body = 29)
    vector_t action_;

    // Configuration (29 DOFs for full G1)
    vector_t defaultJointPos_;  // 29 joints
    vector_t stiffness_;        // 29 joints
    vector_t damping_;          // 29 joints

    std::vector<std::string> jointNames_;  // 29 joint names

    // Observation scales
    scalar_t angVelScale_;
    scalar_t dofVelScale_;
    scalar_t baseHeightScale_;

    // Action processing
    scalar_t actionScale_;
    scalar_t actionClip_;

    // Default base height
    scalar_t defaultBaseHeight_;

    scalar_t rate_;

    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
