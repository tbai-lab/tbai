#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <tbai_bob/BobController.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {

class PyBobController : public tbai::BobController {
   public:
    PyBobController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen,
                    std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::BobController(robotInterfacePtr, refVelGen), postStepCallback_(postStepCallback) {}

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {}
    void atPositions(matrix_t &positions) override {}
    bool ok() const override { return true; }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

}  // namespace tbai

namespace nb = nanobind;

void bind_bob_controller(nb::module_ &m) {
    nb::class_<tbai::PyBobController, tbai::Controller>(m, "BobController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("post_step_callback") = nullptr);
}
