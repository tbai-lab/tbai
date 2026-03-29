#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Rate.hpp>
#include <tbai_core/control/RobotInterface.hpp>

namespace tbai {

typedef tbai::CentralController<tbai::SystemRate<scalar_t>, tbai::SystemTime<std::chrono::high_resolution_clock>>
    CentralControllerPython;

}  // namespace tbai

namespace nb = nanobind;

void bind_central_controller(nb::module_ &m) {
    nb::class_<tbai::CentralControllerPython>(m, "CentralController")
        .def_static("create",
                    [](std::shared_ptr<tbai::RobotInterface> robotInterfacePtr,
                       std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriberPtr) {
                        return std::make_shared<tbai::CentralControllerPython>(robotInterfacePtr,
                                                                               changeControllerSubscriberPtr);
                    })
        .def("start", &tbai::CentralControllerPython::start, nb::call_guard<nb::gil_scoped_release>())
        .def("start_thread", &tbai::CentralControllerPython::startThread, nb::call_guard<nb::gil_scoped_release>())
        .def("stop_thread", &tbai::CentralControllerPython::stopThread, nb::call_guard<nb::gil_scoped_release>())
        .def("add_controller", &tbai::CentralControllerPython::addController, nb::arg("controller"),
             nb::arg("make_active") = false)
        .def("initialize", &tbai::CentralControllerPython::initialize, nb::call_guard<nb::gil_scoped_release>())
        .def("step", &tbai::CentralControllerPython::step, nb::call_guard<nb::gil_scoped_release>())
        .def("get_rate", &tbai::CentralControllerPython::getRate);
}
