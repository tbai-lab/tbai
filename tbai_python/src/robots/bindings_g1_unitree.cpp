#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_G1_UNITREE
#include <tbai_deploy_g1/G1RobotInterfaceUnitree.hpp>
#endif

namespace nb = nanobind;

void bind_g1_unitree(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_G1_UNITREE
    nb::class_<tbai::G1RobotInterfaceUnitreeArgs>(m, "G1RobotInterfaceUnitreeArgs")
        .def(nb::init<>())
        .def_rw("network_interface", &tbai::G1RobotInterfaceUnitreeArgs::networkInterface)
        .def_rw("unitree_channel", &tbai::G1RobotInterfaceUnitreeArgs::unitreeChannel)
        .def_rw("channel_init", &tbai::G1RobotInterfaceUnitreeArgs::channelInit);

    nb::class_<tbai::G1RobotInterfaceUnitree, tbai::RobotInterface>(m, "G1RobotInterfaceUnitree")
        .def(nb::init<tbai::G1RobotInterfaceUnitreeArgs>())
        .def("publish", &tbai::G1RobotInterfaceUnitree::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("waitTillInitialized", &tbai::G1RobotInterfaceUnitree::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("getLatestState", &tbai::G1RobotInterfaceUnitree::getLatestState, nb::call_guard<nb::gil_scoped_release>())
        .def("getBaseQuaternion", &tbai::G1RobotInterfaceUnitree::getBaseQuaternion, nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_G1_UNITREE") =
#ifdef TBAI_HAS_DEPLOY_G1_UNITREE
        true;
#else
        false;
#endif
}
