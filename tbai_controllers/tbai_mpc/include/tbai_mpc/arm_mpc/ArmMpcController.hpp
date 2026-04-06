#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <mutex>
#include <string>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_mpc/arm_mpc/ArmInterface.h>
#include <tbai_mpc/arm_wbc/WbcBase.hpp>

namespace tbai::mpc::arm {

using ocs2::scalar_t;
using ocs2::vector_t;

/**
 * ROS-independent Arm MPC controller.
 * Loads config from env vars TBAI_TASK_FILE_PATH, TBAI_ROBOT_DESCRIPTION_PATH
 * and TBAI_GLOBAL_CONFIG_PATH.
 * EE target is set via setTargetEEPose() (e.g. from a virtual joystick).
 */
class ArmMpcController : public tbai::Controller {
   public:
    ArmMpcController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                     std::function<scalar_t()> getCurrentTimeFunction);

    ~ArmMpcController() override = default;

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    std::string getName() const override { return "ArmMpcController"; }

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override { return controllerType == "MPC"; }

    void stopController() override {}

    scalar_t getRate() const override { return 300.0; }

    bool checkStability() const override { return isStable_; }

    bool ok() const override { return true; }

    void waitTillInitialized() override { robotInterfacePtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = robotInterfacePtr_->getLatestState(); }

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    /** Thread-safe setter for EE target pose, callable from Python/joystick. */
    void setTargetEEPose(const vector_t &position, const vector_t &orientation);

   private:
    void initialize();
    void resetMpc();
    void setObservation();
    ocs2::SystemObservation generateSystemObservation() const;

    std::shared_ptr<tbai::RobotInterface> robotInterfacePtr_;
    std::shared_ptr<spdlog::logger> logger_;

    std::unique_ptr<tbai::mpc::arm::ArmInterface> manipulatorInterfacePtr_;
    std::unique_ptr<WbcBase> wbcPtr_;
    std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpcPtr_;
    std::unique_ptr<ocs2::MPC_MRT_Interface> mrtPtr_;

    scalar_t initTime_ = 0.0;
    scalar_t tNow_ = 0.0;

    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 100.0;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;

    bool isStable_ = true;

    State state_;

    std::function<scalar_t()> getCurrentTimeFunction_;

    // EE target (thread-safe via mutex)
    std::mutex targetMutex_;
    vector_t targetEEPosition_;
    vector_t targetEEOrientation_;
};

}  // namespace tbai::mpc::arm
