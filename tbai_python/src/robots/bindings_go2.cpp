#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <tbai_core/control/RobotInterface.hpp>

#ifdef TBAI_HAS_DEPLOY_GO2
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#endif

#ifdef TBAI_HAS_VO
#include <tbai_estim/inekf/Observations.hpp>
#endif

namespace nb = nanobind;

void bind_go2(nb::module_ &m) {
#ifdef TBAI_HAS_DEPLOY_GO2
    nb::class_<tbai::Go2RobotInterfaceArgs>(m, "Go2RobotInterfaceArgs")
        .def(nb::init<>())
        .def_rw("use_ground_truth_state", &tbai::Go2RobotInterfaceArgs::useGroundTruthState);

    auto cls = nb::class_<tbai::Go2RobotInterface, tbai::RobotInterface>(m, "Go2RobotInterface")
                   .def(nb::init<tbai::Go2RobotInterfaceArgs>())
                   .def("publish", &tbai::Go2RobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
                   .def("wait_till_initialized", &tbai::Go2RobotInterface::waitTillInitialized,
                        nb::call_guard<nb::gil_scoped_release>())
                   .def("get_latest_state", &tbai::Go2RobotInterface::getLatestState,
                        nb::call_guard<nb::gil_scoped_release>());

#ifdef TBAI_HAS_VO
    cls.def("correct_visual_landmarks", &tbai::Go2RobotInterface::correctVisualLandmarks, nb::arg("landmarks"),
            nb::arg("prune_stale") = true, nb::call_guard<nb::gil_scoped_release>(),
            "Forward visual-odometry landmark observations to the internal InEKF. "
            "If prune_stale=True (default), drops any tracked landmark not in this batch -- "
            "essential to keep the state matrix bounded.");
#endif
#endif

    m.attr("HAS_DEPLOY_GO2") =
#ifdef TBAI_HAS_DEPLOY_GO2
        true;
#else
        false;
#endif
}
