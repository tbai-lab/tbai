#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

#ifdef TBAI_HAS_DEPLOY_G1
#include <tbai_deploy_g1/G1ASAPController.hpp>
#include <tbai_deploy_g1/G1ASAPMimicController.hpp>
#include <tbai_deploy_g1/G1BeyondMimicController.hpp>
#include <tbai_deploy_g1/G1MimicController.hpp>
#include <tbai_deploy_g1/G1PBHCController.hpp>
#include <tbai_deploy_g1/G1RLController.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_deploy_g1/G1SpinkickController.hpp>
#include <tbai_deploy_g1/G1Twist2Controller.hpp>
#endif

namespace nb = nanobind;

void bind_g1(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_G1
    nb::class_<tbai::G1RobotInterfaceArgs>(m, "G1RobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("use_ground_truth_state", &tbai::G1RobotInterfaceArgs::useGroundTruthState);

    nb::class_<tbai::G1RobotInterface, tbai::RobotInterface>(m, "G1RobotInterface")
        .def(nb::init<tbai::G1RobotInterfaceArgs>())
        .def("publish", &tbai::G1RobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::G1RobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::G1RobotInterface::getLatestState, nb::call_guard<nb::gil_scoped_release>())
        .def("get_base_quaternion", &tbai::G1RobotInterface::getBaseQuaternion, nb::call_guard<nb::gil_scoped_release>());

    nb::class_<tbai::g1::G1RLController, tbai::Controller>(m, "G1RLController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("policy_path"));

    nb::class_<tbai::g1::G1MimicController, tbai::Controller>(m, "G1MimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("motion_fps") = 60.0f, nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f,
             nb::arg("controller_name") = "G1MimicController")
        .def("is_motion_complete", &tbai::g1::G1MimicController::isMotionComplete)
        .def("get_motion_time", &tbai::g1::G1MimicController::getMotionTime);

    nb::class_<tbai::g1::G1ASAPController, tbai::Controller>(m, "G1ASAPController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &,
                      const std::string &>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("policy_path"),
             nb::arg("controller_name") = "G1ASAPLocomotion");

    nb::class_<tbai::g1::G1ASAPMimicController, tbai::Controller>(m, "G1ASAPMimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_length"),
             nb::arg("controller_name") = "G1ASAPMimic")
        .def("is_motion_complete", &tbai::g1::G1ASAPMimicController::isMotionComplete)
        .def("get_motion_phase", &tbai::g1::G1ASAPMimicController::getMotionPhase);

    nb::class_<tbai::g1::G1PBHCController, tbai::Controller>(m, "G1PBHCController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f, nb::arg("controller_name") = "G1PBHCController")
        .def("is_motion_complete", &tbai::g1::G1PBHCController::isMotionComplete)
        .def("get_motion_time", &tbai::g1::G1PBHCController::getMotionTime);

    nb::class_<tbai::g1::G1Twist2Controller, tbai::Controller>(m, "G1Twist2Controller")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f,
             nb::arg("controller_name") = "G1Twist2Controller")
        .def("is_motion_complete", &tbai::g1::G1Twist2Controller::isMotionComplete)
        .def("get_motion_time", &tbai::g1::G1Twist2Controller::getMotionTime);

    nb::class_<tbai::g1::G1SpinkickController, tbai::Controller>(m, "G1SpinkickController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("controller_name") = "G1SpinkickController",
             nb::arg("use_model_meta_config") = true, nb::arg("action_beta") = 1.0f)
        .def("is_motion_complete", &tbai::g1::G1SpinkickController::isMotionComplete)
        .def("get_timestep", &tbai::g1::G1SpinkickController::getTimestep)
        .def("get_max_timestep", &tbai::g1::G1SpinkickController::getMaxTimestep);

    nb::class_<tbai::g1::G1BeyondMimicController, tbai::Controller>(m, "G1BeyondMimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("controller_name") = "G1BeyondMimicController",
             nb::arg("use_model_meta_config") = true, nb::arg("action_beta") = 1.0f)
        .def("is_motion_complete", &tbai::g1::G1BeyondMimicController::isMotionComplete)
        .def("get_timestep", &tbai::g1::G1BeyondMimicController::getTimestep)
        .def("get_max_timestep", &tbai::g1::G1BeyondMimicController::getMaxTimestep);
#endif

    m.attr("HAS_DEPLOY_G1") =
#ifdef TBAI_HAS_DEPLOY_G1
        true;
#else
        false;
#endif
}
