#include <algorithm>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/RobotInterface.hpp>

namespace tbai {

class PythonController : public tbai::Controller {
   public:
    PythonController(std::shared_ptr<tbai::RobotInterface> robotInterfacePtr, std::string name, scalar_t rate,
                     std::vector<std::string> supportedControllers)
        : robotInterfacePtr_(std::move(robotInterfacePtr)),
          name_(std::move(name)),
          rate_(rate),
          supportedControllers_(std::move(supportedControllers)) {}

    ~PythonController() override = default;

    // Pure virtual — must be overridden in Python.
    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override = 0;

    std::string getName() const override { return name_; }
    scalar_t getRate() const override { return rate_; }
    bool isSupported(const std::string &controllerType) override {
        return std::find(supportedControllers_.begin(), supportedControllers_.end(), controllerType) !=
               supportedControllers_.end();
    }

    // No-op defaults for the rest — Python users rarely need these.
    void changeController(const std::string &, scalar_t) override {}
    void stopController() override {}
    bool checkStability() const override { return true; }
    bool ok() const override { return true; }
    void waitTillInitialized() override { robotInterfacePtr_->waitTillInitialized(); }

    void preStep(scalar_t, scalar_t) override { state_ = robotInterfacePtr_->getLatestState(); }

    // Accessible from Python so the override can read the cached state.
    const State &latestState() const { return state_; }

   protected:
    std::shared_ptr<tbai::RobotInterface> robotInterfacePtr_;
    State state_;

   private:
    std::string name_;
    scalar_t rate_;
    std::vector<std::string> supportedControllers_;
};

class PyPythonController : public PythonController {
   public:
    PyPythonController(std::shared_ptr<tbai::RobotInterface> robotInterfacePtr, std::string name, scalar_t rate,
                       std::vector<std::string> supportedControllers)
        : PythonController(std::move(robotInterfacePtr), std::move(name), rate, std::move(supportedControllers)) {}

    NB_TRAMPOLINE(PythonController, 1);

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override {
        NB_OVERRIDE_PURE_NAME("get_motor_commands", getMotorCommands, currentTime, dt);
    }
};

}  // namespace tbai

namespace nb = nanobind;

void bind_python_controller(nb::module_ &m) {
    nb::class_<tbai::PythonController, tbai::Controller, tbai::PyPythonController>(m, "PythonController")
        .def(nb::init<std::shared_ptr<tbai::RobotInterface>, std::string, tbai::scalar_t,
                      std::vector<std::string>>(),
             nb::arg("robot_interface"), nb::arg("name"), nb::arg("rate") = 100.0,
             nb::arg("supported_controllers") = std::vector<std::string>{})
        .def("get_motor_commands", &tbai::PythonController::getMotorCommands, nb::arg("current_time"),
             nb::arg("dt"))
        .def("get_latest_state", &tbai::PythonController::latestState, nb::rv_policy::reference_internal);
}
