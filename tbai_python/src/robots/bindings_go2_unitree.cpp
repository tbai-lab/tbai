#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_GO2_UNITREE
#include <tbai_deploy_go2/Go2RobotInterfaceUnitree.hpp>
#endif

namespace nb = nanobind;

void bind_go2_unitree(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_GO2_UNITREE
    nb::class_<tbai::Go2RobotInterfaceUnitreeArgs>(m, "Go2RobotInterfaceUnitreeArgs")
        .def(nb::init<>())
        .def_rw("network_interface", &tbai::Go2RobotInterfaceUnitreeArgs::networkInterface)
        .def_rw("unitree_channel", &tbai::Go2RobotInterfaceUnitreeArgs::unitreeChannel)
        .def_rw("channel_init", &tbai::Go2RobotInterfaceUnitreeArgs::channelInit);

    nb::class_<tbai::Go2RobotInterfaceUnitree, tbai::RobotInterface>(m, "Go2RobotInterfaceUnitree")
        .def(nb::init<tbai::Go2RobotInterfaceUnitreeArgs>())
        .def("publish", &tbai::Go2RobotInterfaceUnitree::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::Go2RobotInterfaceUnitree::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::Go2RobotInterfaceUnitree::getLatestState, nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_GO2_UNITREE") =
#ifdef TBAI_HAS_DEPLOY_GO2_UNITREE
        true;
#else
        false;
#endif
}
