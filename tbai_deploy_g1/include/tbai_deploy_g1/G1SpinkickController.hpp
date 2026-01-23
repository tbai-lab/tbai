#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_deploy_g1/G1Constants.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief G1 Spinkick Controller - skill-based motion controller using ONNX.
 *
 * This controller executes a spinkick skill motion using a pre-trained ONNX model.
 * It follows the same pattern as G1BeyondMimicController but is designed for
 * specific skill execution with a fixed timestep limit.
 */
class G1SpinkickController : public tbai::Controller {
   public:
    G1SpinkickController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr, const std::string &policyPath,
                         const std::string &controllerName = "G1SpinkickController", bool useModelMetaConfig = true,
                         float actionBeta = 1.0f);

    ~G1SpinkickController();

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return controllerName_; }

    void stopController() override;

    bool ok() const override;

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isMotionComplete() const;

    int getTimestep() const { return timestep_; }

    int getMaxTimestep() const { return maxTimestep_; }

   protected:
    void initOnnxRuntime(const std::string &policyPath);
    void parseModelMetadata();
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
    std::vector<const char *> inputNames_;
    std::vector<const char *> outputNames_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    vector_t observation_;

    // Action output
    vector_t action_;
    vector_t rawAction_;

    // Motion data from ONNX outputs
    vector_t motionJointPos_;
    vector_t motionJointVel_;

    // Configuration
    vector_t defaultJointPos_;
    vector_t actionScale_;
    vector_t stiffness_;
    vector_t damping_;
    std::vector<int> jointIdsMap_;

    std::vector<std::string> jointNames_;

    float actionBeta_;
    bool useModelMetaConfig_;

    int timestep_;
    int maxTimestep_;
    bool motionActive_;

    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
