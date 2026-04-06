#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_ANYMAL_C
#include <tbai_deploy_anymal_c/AnymalCRobotInterface.hpp>
#endif

namespace nb = nanobind;

void bind_anymal_c(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_ANYMAL_C
    nb::class_<tbai::AnymalCRobotInterfaceArgs>(m, "AnymalCRobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("use_ground_truth_state", &tbai::AnymalCRobotInterfaceArgs::useGroundTruthState);

    nb::class_<tbai::AnymalCRobotInterface, tbai::RobotInterface>(m, "AnymalCRobotInterface")
        .def(nb::init<tbai::AnymalCRobotInterfaceArgs>())
        .def("publish", &tbai::AnymalCRobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::AnymalCRobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::AnymalCRobotInterface::getLatestState,
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_ANYMAL_C") =
#ifdef TBAI_HAS_DEPLOY_ANYMAL_C
        true;
#else
        false;
#endif
}
