// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <fstream>

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <tbai_core/Types.hpp>
#include <tbai_core/control/Rate.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

#ifdef TBAI_HAS_QUADRUPED_MPC
#include <tbai_mpc/quadruped_mpc/MpcController.hpp>
#endif

namespace tbai {

#ifdef TBAI_HAS_QUADRUPED_MPC
class PyQuadrupedMpcController : public tbai::mpc::quadruped::MpcController {
   public:
    PyQuadrupedMpcController(const std::string &robotName,
                             const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                             std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
                             const std::string &urdfPath, const std::string &taskSettingsFile,
                             const std::string &frameDeclarationFile, const std::string &controllerConfigFile,
                             const std::string &targetCommandFile, const std::string &sqpSettingsFile = "",
                             const std::string &gaitFile = "", scalar_t trajdt = 0.1, size_t trajKnots = 20,
                             std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::mpc::quadruped::MpcController(
              robotName, robotInterfacePtr, std::move(velocityGeneratorPtr),
              &tbai::SystemTime<std::chrono::high_resolution_clock>::rightNow),
          postStepCallback_(postStepCallback) {
        // Read URDF file
        std::ifstream urdfStream(urdfPath);
        std::string urdfString((std::istreambuf_iterator<char>(urdfStream)), std::istreambuf_iterator<char>());

        initialize(urdfString, taskSettingsFile, frameDeclarationFile, controllerConfigFile, targetCommandFile, trajdt,
                   trajKnots, sqpSettingsFile, gaitFile);
    }

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

void bind_quadruped_mpc_controller(nb::module_ &m) {
#ifdef TBAI_HAS_QUADRUPED_MPC
    nb::class_<tbai::PyQuadrupedMpcController, tbai::Controller>(m, "QuadrupedMpcController")
        .def(nb::init<const std::string &, const std::shared_ptr<tbai::RobotInterface> &,
                      std::shared_ptr<tbai::reference::ReferenceVelocityGenerator>, const std::string &,
                      const std::string &, const std::string &, const std::string &, const std::string &,
                      const std::string &, const std::string &, tbai::scalar_t, size_t,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_name"), nb::arg("robot_interface"), nb::arg("velocity_generator"), nb::arg("urdf_path"),
             nb::arg("task_settings_file"), nb::arg("frame_declaration_file"), nb::arg("controller_config_file"),
             nb::arg("target_command_file"), nb::arg("sqp_settings_file") = "", nb::arg("gait_file") = "",
             nb::arg("traj_dt") = 0.1, nb::arg("traj_knots") = 20, nb::arg("post_step_callback") = nullptr)
        .def("set_gait", &tbai::PyQuadrupedMpcController::setGait, nb::arg("gait_name"),
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_QUADRUPED_MPC") =
#ifdef TBAI_HAS_QUADRUPED_MPC
        true;
#else
        false;
#endif
}
