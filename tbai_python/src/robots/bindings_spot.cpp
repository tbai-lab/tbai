#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_SPOT
#include <tbai_deploy_spot/SpotRobotInterface.hpp>
#endif

namespace nb = nanobind;

void bind_spot(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_SPOT
    nb::class_<tbai::SpotRobotInterfaceArgs>(m, "SpotRobotInterfaceArgs")
        .def(nb::init<>());

    nb::class_<tbai::SpotRobotInterface, tbai::RobotInterface>(m, "SpotRobotInterface")
        .def(nb::init<tbai::SpotRobotInterfaceArgs>())
        .def("publish", &tbai::SpotRobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::SpotRobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::SpotRobotInterface::getLatestState,
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_SPOT") =
#ifdef TBAI_HAS_DEPLOY_SPOT
        true;
#else
        false;
#endif
}
