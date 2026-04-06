// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_mpc/arm_mpc/ArmMpcController.hpp"

#include <fstream>

#include <tbai_core/Env.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/arm_wbc/Factory.hpp>

namespace tbai::mpc::arm {

ArmMpcController::ArmMpcController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                                   std::function<scalar_t()> getCurrentTimeFunction)
    : robotInterfacePtr_(robotInterfacePtr), getCurrentTimeFunction_(getCurrentTimeFunction) {
    logger_ = tbai::getLogger("arm_mpc_controller");
    initTime_ = tbai::readInitTime();
    initialize();
}

void ArmMpcController::initialize() {
    auto taskFile = tbai::getEnvAs<std::string>("TBAI_TASK_FILE_PATH");
    auto libFolder = tbai::fromGlobalConfig<std::string>("arm_mpc/lib_folder");
    auto urdfFile = tbai::getEnvAs<std::string>("TBAI_ROBOT_DESCRIPTION_PATH");

    TBAI_LOG_INFO(logger_, "Loading task file: {}", taskFile);
    TBAI_LOG_INFO(logger_, "Loading library folder: {}", libFolder);
    TBAI_LOG_INFO(logger_, "Loading URDF file: {}", urdfFile);

    manipulatorInterfacePtr_ = std::make_unique<tbai::mpc::arm::ArmInterface>(taskFile, libFolder, urdfFile);

    std::ifstream urdfStream(urdfFile);
    std::string urdfString((std::istreambuf_iterator<char>(urdfStream)), std::istreambuf_iterator<char>());

    wbcPtr_ = getWbcUnique(taskFile, urdfString, manipulatorInterfacePtr_->getArmModelInfo());

    mpcPtr_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
        manipulatorInterfacePtr_->mpcSettings(), manipulatorInterfacePtr_->ddpSettings(),
        manipulatorInterfacePtr_->getRollout(), manipulatorInterfacePtr_->getOptimalControlProblem(),
        manipulatorInterfacePtr_->getInitializer());
    mpcPtr_->getSolverPtr()->setReferenceManager(manipulatorInterfacePtr_->getReferenceManagerPtr());

    mrtPtr_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpcPtr_, true);

    // Default target EE pose
    targetEEPosition_ = vector_t::Zero(3);
    targetEEPosition_(0) = 0.4;
    targetEEPosition_(2) = 0.3;
    targetEEOrientation_ = vector_t::Zero(4);
    targetEEOrientation_(1) = 1.0;  // 180 deg around Y (pointing down), xyzw

    mpcRate_ = manipulatorInterfacePtr_->mpcSettings().mpcDesiredFrequency_;
    tNow_ = 0.0;
    TBAI_LOG_INFO(logger_, "ArmMpcController initialized (MPC rate: {} Hz)", mpcRate_);
}

std::vector<MotorCommand> ArmMpcController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    mrtPtr_->spinMRT();
    mrtPtr_->updatePolicy();

    tNow_ = getCurrentTimeFunction_() - initTime_;

    auto observation = generateSystemObservation();

    vector_t targetPos, targetOri;
    {
        std::lock_guard<std::mutex> lock(targetMutex_);
        targetPos = targetEEPosition_;
        targetOri = targetEEOrientation_;
    }

    vector_t desiredState, desiredInput;
    size_t desiredMode;
    mrtPtr_->evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);

    const size_t nJoints = manipulatorInterfacePtr_->getArmModelInfo().armDim;
    vector_t desiredJointAcceleration = vector_t::Zero(nJoints);

    auto commands = wbcPtr_->getMotorCommands(tNow_, observation.state, observation.input, desiredState, desiredInput,
                                              desiredJointAcceleration, targetPos, targetOri, isStable_);

    timeSinceLastMpcUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_) {
        vector_t targetState(7);
        targetState.head(3) = targetPos;
        targetState.tail(4) = targetOri;
        const vector_t zeroInput = vector_t::Zero(manipulatorInterfacePtr_->getArmModelInfo().inputDim);
        const scalar_t horizon = 2.0;
        ocs2::TargetTrajectories targetTrajectories({observation.time, observation.time + horizon},
                                                    {targetState, targetState}, {zeroInput, zeroInput});
        mrtPtr_->setTargetTrajectories(targetTrajectories);
        setObservation();
    }

    return commands;
}

void ArmMpcController::setTargetEEPose(const vector_t &position, const vector_t &orientation) {
    std::lock_guard<std::mutex> lock(targetMutex_);
    targetEEPosition_ = position;
    targetEEOrientation_ = orientation;
}

void ArmMpcController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);

    if (!mrt_initialized_ || currentTime + 0.1 > mrtPtr_->getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = currentTime - initTime_;
}

void ArmMpcController::resetMpc() {
    robotInterfacePtr_->waitTillInitialized();
    auto initialObservation = generateSystemObservation();

    vector_t targetPos, targetOri;
    {
        std::lock_guard<std::mutex> lock(targetMutex_);
        targetPos = targetEEPosition_;
        targetOri = targetEEOrientation_;
    }

    vector_t initTarget(7);
    initTarget.head(3) = targetPos;
    initTarget.tail(4) = targetOri;
    const vector_t zeroInput = vector_t::Zero(manipulatorInterfacePtr_->getArmModelInfo().inputDim);
    const ocs2::TargetTrajectories initTargetTrajectories({initialObservation.time}, {initTarget}, {zeroInput});
    mrtPtr_->resetMpcNode(initTargetTrajectories);

    while (!mrtPtr_->initialPolicyReceived()) {
        TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
        initialObservation = generateSystemObservation();
        mrtPtr_->setCurrentObservation(initialObservation);
        mrtPtr_->spinMRT();
        mrtPtr_->updatePolicy();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    TBAI_LOG_INFO(logger_, "Initial policy received.");
}

void ArmMpcController::setObservation() {
    mrtPtr_->setCurrentObservation(generateSystemObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

ocs2::SystemObservation ArmMpcController::generateSystemObservation() const {
    auto state = robotInterfacePtr_->getLatestState();
    const tbai::vector_t &rbdState = state.x;
    const size_t nJoints = manipulatorInterfacePtr_->getArmModelInfo().armDim;

    ocs2::SystemObservation observation;
    observation.time = state.timestamp - initTime_;
    observation.mode = 0;
    observation.state = rbdState.head(nJoints);
    observation.input = rbdState.segment(nJoints, nJoints);

    return observation;
}

}  // namespace tbai::mpc::arm
