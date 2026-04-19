#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_FRANKA
#include <tbai_deploy_franka/FrankaRobotInterface.hpp>
#endif

namespace nb = nanobind;

void bind_franka(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_FRANKA
    nb::class_<tbai::FrankaRobotInterfaceArgs>(m, "FrankaRobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("close_gripper", &tbai::FrankaRobotInterfaceArgs::closeGripper);

    nb::class_<tbai::FrankaRobotInterface, tbai::RobotInterface>(m, "FrankaRobotInterface")
        .def(nb::init<tbai::FrankaRobotInterfaceArgs>())
        .def("publish", &tbai::FrankaRobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::FrankaRobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::FrankaRobotInterface::getLatestState,
             nb::call_guard<nb::gil_scoped_release>())
        .def("set_finger_command", &tbai::FrankaRobotInterface::setFingerCommand, nb::arg("desired_position"),
             nb::arg("desired_velocity") = 0.0, nb::arg("kp") = tbai::FRANKA_GRIPPER_DEFAULT_KP,
             nb::arg("kd") = tbai::FRANKA_GRIPPER_DEFAULT_KD, nb::arg("torque_ff") = 0.0,
             nb::call_guard<nb::gil_scoped_release>())
        .def("open_gripper", &tbai::FrankaRobotInterface::openGripper,
             nb::arg("kp") = tbai::FRANKA_GRIPPER_DEFAULT_KP, nb::arg("kd") = tbai::FRANKA_GRIPPER_DEFAULT_KD,
             nb::call_guard<nb::gil_scoped_release>())
        .def("close_gripper", &tbai::FrankaRobotInterface::closeGripper,
             nb::arg("kp") = tbai::FRANKA_GRIPPER_DEFAULT_KP, nb::arg("kd") = tbai::FRANKA_GRIPPER_DEFAULT_KD,
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_FRANKA") =
#ifdef TBAI_HAS_DEPLOY_FRANKA
        true;
#else
        false;
#endif
}
