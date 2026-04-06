// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>

#include <tbai_core/Types.hpp>
#include <tbai_core/control/Rate.hpp>
#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_ARM_MPC
#include <tbai_mpc/arm_mpc/ArmMpcController.hpp>
#endif

namespace tbai {

#ifdef TBAI_HAS_ARM_MPC
class PyArmMpcController : public tbai::mpc::arm::ArmMpcController {
   public:
    PyArmMpcController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                       std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::mpc::arm::ArmMpcController(
              robotInterfacePtr, &tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow),
          postStepCallback_(postStepCallback) {}

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};
#endif

}  // namespace tbai

namespace nb = nanobind;

void bind_arm_mpc_controller(nb::module_ &m) {
#ifdef TBAI_HAS_ARM_MPC
    nb::class_<tbai::PyArmMpcController, tbai::Controller>(m, "ArmMpcController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("post_step_callback") = nullptr)
        .def(
            "set_target_ee_pose",
            [](tbai::PyArmMpcController &self, const Eigen::Vector3d &position,
               const Eigen::Vector4d &orientation) {
                ocs2::vector_t pos(3);
                pos << position.x(), position.y(), position.z();
                ocs2::vector_t ori(4);
                ori << orientation.x(), orientation.y(), orientation.z(), orientation.w();
                self.setTargetEEPose(pos, ori);
            },
            nb::arg("position"), nb::arg("orientation"), nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_ARM_MPC") =
#ifdef TBAI_HAS_ARM_MPC
        true;
#else
        false;
#endif
}
