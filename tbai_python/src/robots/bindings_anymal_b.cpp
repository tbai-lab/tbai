#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_ANYMAL_B
#include <tbai_deploy_anymal_b/AnymalBRobotInterface.hpp>
#endif

namespace nb = nanobind;

void bind_anymal_b(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_ANYMAL_B
    nb::class_<tbai::AnymalBRobotInterfaceArgs>(m, "AnymalBRobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("use_ground_truth_state", &tbai::AnymalBRobotInterfaceArgs::useGroundTruthState);

    nb::class_<tbai::AnymalBRobotInterface, tbai::RobotInterface>(m, "AnymalBRobotInterface")
        .def(nb::init<tbai::AnymalBRobotInterfaceArgs>())
        .def("publish", &tbai::AnymalBRobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("wait_till_initialized", &tbai::AnymalBRobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("get_latest_state", &tbai::AnymalBRobotInterface::getLatestState,
             nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_ANYMAL_B") =
#ifdef TBAI_HAS_DEPLOY_ANYMAL_B
        true;
#else
        false;
#endif
}
