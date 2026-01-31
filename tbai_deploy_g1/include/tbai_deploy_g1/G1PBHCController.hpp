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
#include <tbai_deploy_g1/PBHCMotionLoader.hpp>

// ONNX Runtime
#include <onnxruntime_cxx_api.h>

namespace tbai {
namespace g1 {

// PBHC Controller Constants (23 DOFs, locked wrists)
constexpr int PBHC_NUM_JOINTS = 23;
constexpr int PBHC_HISTORY_LENGTH = 4;  // 4 frames of history

// Current observation components (before history): actions(23) + ang_vel(3) + dof_pos(23) + dof_vel(23) = 72
// History: 304 dims (see below)
// After history: gravity(3) + phase(1) = 4
// Total current: 72 + 4 = 76
constexpr int PBHC_CURRENT_OBS_SIZE = 76;

// History observation (sorted alphabetically by type):
// actions(4*23) + base_ang_vel(4*3) + dof_pos(4*23) + dof_vel(4*23) + projected_gravity(4*3) + ref_motion_phase(4*1)
// = 92 + 12 + 92 + 92 + 12 + 4 = 304
constexpr int PBHC_HISTORY_OBS_SIZE = 304;

// Total observation: current(76) + history(304) = 380
constexpr int PBHC_TOTAL_OBS_SIZE = PBHC_CURRENT_OBS_SIZE + PBHC_HISTORY_OBS_SIZE;

/**
 * @brief G1 PBHC Controller for motion tracking using ONNX Runtime.
 *
 * This controller implements the PBHC (Perpetual Balance Humanoid Control) motion
 * tracking algorithm. It uses a neural network policy to track motion from
 * PKL motion files and outputs joint position targets for 23 DOFs (locked wrists).
 *
 * Observation structure (380 dims, ALPHABETICALLY SORTED):
 * 1. actions (current): 23
 * 2. base_ang_vel (current): 3
 * 3. dof_pos (current): 23
 * 4. dof_vel (current): 23
 * 5. history_actor (304):
 *    - actions (4 * 23 = 92)
 *    - base_ang_vel (4 * 3 = 12)
 *    - dof_pos (4 * 23 = 92)
 *    - dof_vel (4 * 23 = 92)
 *    - projected_gravity (4 * 3 = 12)
 *    - ref_motion_phase (4 * 1 = 4)
 * 6. projected_gravity (current): 3
 * 7. ref_motion_phase (current): 1
 */
class G1PBHCController : public tbai::Controller {
   public:
    G1PBHCController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr, const std::string &policyPath,
                     const std::string &motionFilePath, float timeStart = 0.0f, float timeEnd = -1.0f,
                     const std::string &controllerName = "G1PBHCController");

    ~G1PBHCController();

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
    void updateHistory();

    /**
     * @brief Get projected gravity vector from quaternion
     */
    vector_t getProjectedGravity(const vector4_t &quat) const;

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::unique_ptr<PBHCMotionLoader> motionLoader_;

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

    // Full observation (380 dims)
    vector_t observation_;

    // Current observation components (before scaling)
    vector_t currentAngVel_;   // 3 dims
    vector_t currentGravity_;  // 3 dims
    vector_t currentDofPos_;   // 23 dims (relative to default)
    vector_t currentDofVel_;   // 23 dims
    vector_t currentActions_;  // 23 dims
    scalar_t currentPhase_;    // 1 dim

    // History buffers (each is a deque of PBHC_HISTORY_LENGTH vectors)
    std::deque<vector_t> historyActions_;  // 4 x 23
    std::deque<vector_t> historyAngVel_;   // 4 x 3
    std::deque<vector_t> historyDofPos_;   // 4 x 23
    std::deque<vector_t> historyDofVel_;   // 4 x 23
    std::deque<vector_t> historyGravity_;  // 4 x 3
    std::deque<scalar_t> historyPhase_;    // 4 x 1

    // Action output
    vector_t action_;

    // Configuration (23 DOFs)
    vector_t defaultJointPos_;  // 23 joints
    vector_t stiffness_;        // 23 joints
    vector_t damping_;          // 23 joints

    // Joint mapping from 23-DOF PBHC order to 29-DOF G1 order
    std::vector<int> jointMapping_;

    std::vector<std::string> jointNames_;  // 29 joint names (G1 full)

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
