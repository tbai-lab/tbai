#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_ANYMAL_D
#include <tbai_deploy_anymal_d/AnymalDRobotInterface.hpp>
#endif

namespace nb = nanobind;

void bind_anymal_d(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_ANYMAL_D
    nb::class_<tbai::AnymalDRobotInterfaceArgs>(m, "AnymalDRobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("use_ground_truth_state", &tbai::AnymalDRobotInterfaceArgs::useGroundTruthState);

    nb::class_<tbai::AnymalDRobotInterface, tbai::RobotInterface>(m, "AnymalDRobotInterface")
        .def(nb::init<tbai::AnymalDRobotInterfaceArgs>())
        .def("publish", &tbai::AnymalDRobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::AnymalDRobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::AnymalDRobotInterface::getLatestState,
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_ANYMAL_D") =
#ifdef TBAI_HAS_DEPLOY_ANYMAL_D
        true;
#else
        false;
#endif
}
