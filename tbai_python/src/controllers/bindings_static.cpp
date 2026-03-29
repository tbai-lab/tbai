#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include <tbai_core/Types.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_static/StaticController.hpp>

namespace tbai {

class PyStaticController : public tbai::static_::StaticController {
   public:
    PyStaticController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                       std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::static_::StaticController(robotInterfacePtr), postStepCallback_(postStepCallback) {}

    bool ok() const override { return true; }
    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = robotInterfacePtr_->getLatestState(); }

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

}  // namespace tbai

namespace nb = nanobind;

void bind_static_controller(nb::module_ &m) {
    nb::class_<tbai::PyStaticController, tbai::Controller>(m, "StaticController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("post_step_callback") = nullptr);
}
