#include <cstring>

// nanobind
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

// tbai
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {

class PyRobotInterface : public tbai::RobotInterface {
   public:
    NB_TRAMPOLINE(tbai::RobotInterface, 3);
    void waitTillInitialized() override { NB_OVERRIDE_PURE_NAME("wait_till_initialized", waitTillInitialized); }
    State getLatestState() override { NB_OVERRIDE_PURE_NAME("get_latest_state", getLatestState); }
    void publish(std::vector<MotorCommand> commands) override { NB_OVERRIDE_PURE(publish, commands); }
};

class PyChangeControllerSubscriber : public tbai::ChangeControllerSubscriber {
   public:
    NB_TRAMPOLINE(tbai::ChangeControllerSubscriber, 2);

    void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) override {
        NB_OVERRIDE_NAME("set_callback_function", setCallbackFunction, callbackFunction);
    }
    void triggerCallbacks() override { NB_OVERRIDE_PURE_NAME("trigger_callbacks", triggerCallbacks); }
};

class PyReferenceVelocityGenerator : public tbai::reference::ReferenceVelocityGenerator {
   public:
    NB_TRAMPOLINE(tbai::reference::ReferenceVelocityGenerator, 1);

    tbai::reference::ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override {
        NB_OVERRIDE_PURE_NAME("get_reference_velocity", getReferenceVelocity, time, dt);
    }
};

}  // namespace tbai

namespace nb = nanobind;

// Forward declarations — robots
void bind_go2(nb::module_ &m);
void bind_go2_unitree(nb::module_ &m);
void bind_g1(nb::module_ &m);
void bind_g1_unitree(nb::module_ &m);
void bind_franka(nb::module_ &m);
void bind_anymal_b(nb::module_ &m);
void bind_anymal_c(nb::module_ &m);
void bind_anymal_d(nb::module_ &m);
void bind_spot(nb::module_ &m);

// Forward declarations — controllers
void bind_static_controller(nb::module_ &m);
void bind_bob_controller(nb::module_ &m);
void bind_np3o_controller(nb::module_ &m);
void bind_wtw_controller(nb::module_ &m);
void bind_central_controller(nb::module_ &m);
void bind_arm_mpc_controller(nb::module_ &m);
void bind_quadruped_mpc_controller(nb::module_ &m);
void bind_python_controller(nb::module_ &m);

NB_MODULE(_C, m) {
    nb::set_leak_warnings(false);

    m.def("write_init_time", nb::overload_cast<>(&tbai::writeInitTime));
    m.def("write_init_time", nb::overload_cast<const long, const long>(&tbai::writeInitTime));
    m.def("write_init_time", nb::overload_cast<const double>(&tbai::writeInitTime));
    m.def("read_init_time", &tbai::readInitTime);
    m.def("download_from_huggingface", &tbai::downloadFromHuggingFace, nb::arg("repo_id"), nb::arg("filename"),
          nb::call_guard<nb::gil_scoped_release>());

    nb::class_<tbai::MotorCommand>(m, "MotorCommand")
        .def(nb::init<>())
        .def("__init__",
             [](tbai::MotorCommand *self, const std::string &joint_name, double desired_position,
                double desired_velocity, double kp, double kd, double torque_ff) {
                 new (self) tbai::MotorCommand();
                 self->joint_name = joint_name;
                 self->desired_position = desired_position;
                 self->desired_velocity = desired_velocity;
                 self->kp = kp;
                 self->kd = kd;
                 self->torque_ff = torque_ff;
             },
             nb::arg("joint_name"), nb::arg("desired_position") = 0.0, nb::arg("desired_velocity") = 0.0,
             nb::arg("kp") = 0.0, nb::arg("kd") = 0.0, nb::arg("torque_ff") = 0.0)
        .def_rw("kp", &tbai::MotorCommand::kp)
        .def_rw("desired_position", &tbai::MotorCommand::desired_position)
        .def_rw("kd", &tbai::MotorCommand::kd)
        .def_rw("desired_velocity", &tbai::MotorCommand::desired_velocity)
        .def_rw("torque_ff", &tbai::MotorCommand::torque_ff)
        .def_rw("joint_name", &tbai::MotorCommand::joint_name);

    nb::class_<tbai::State>(m, "State")
        .def(nb::init<>())
        .def_rw("x", &tbai::State::x)
        .def_rw("timestamp", &tbai::State::timestamp)
        .def_rw("contact_flags", &tbai::State::contactFlags);

    nb::class_<tbai::RobotInterface, tbai::PyRobotInterface>(m, "RobotInterface")
        .def(nb::init<>())
        .def("wait_till_initialized", &tbai::RobotInterface::waitTillInitialized)
        .def("get_latest_state", &tbai::RobotInterface::getLatestState)
        .def("publish", &tbai::RobotInterface::publish);

    nb::class_<tbai::ChangeControllerSubscriber, tbai::PyChangeControllerSubscriber>(m, "ChangeControllerSubscriber")
        .def(nb::init<>())
        .def("set_callback_function", &tbai::ChangeControllerSubscriber::setCallbackFunction)
        .def("trigger_callbacks", &tbai::ChangeControllerSubscriber::triggerCallbacks);

    nb::class_<tbai::reference::ReferenceVelocity>(m, "ReferenceVelocity")
        .def(nb::init<>())
        .def_rw("velocity_x", &tbai::reference::ReferenceVelocity::velocity_x)
        .def_rw("velocity_y", &tbai::reference::ReferenceVelocity::velocity_y)
        .def_rw("yaw_rate", &tbai::reference::ReferenceVelocity::yaw_rate);

    nb::class_<tbai::reference::ReferenceVelocityGenerator, tbai::PyReferenceVelocityGenerator>(m, "ReferenceVelocityGenerator")
        .def(nb::init<>())
        .def("get_reference_velocity", &tbai::reference::ReferenceVelocityGenerator::getReferenceVelocity);

    // Bind rotation helper functions
    nb::module_ rotations_module = m.def_submodule("rotations");
    rotations_module.def(
        "rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        nb::arg("rpy"), "Convert roll-pitch-yaw euler angles to quaternion (returns xyzw vector)");
    rotations_module.def(
        "quat2mat",
        [](const tbai::vector4_t &q) {
            tbai::matrix3_t mat = tbai::quat2mat(tbai::quaternion_t(q[3], q[0], q[1], q[2]));
            return tbai::matrix3_t(mat);
        },
        nb::arg("quat_xyzw"), "Convert quaternion to rotation matrix, expects xyzw vector");
    rotations_module.def("mat2rpy", &tbai::mat2rpy, nb::arg("mat"),
                         "Convert rotation matrix to roll-pitch-yaw euler angles");
    rotations_module.def("mat2ocs2rpy", &tbai::mat2oc2rpy, nb::arg("mat"), nb::arg("last_yaw"),
                         "Convert rotation matrix to ocs2-style roll-pitch-yaw euler angles");
    rotations_module.def(
        "ocs2rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::ocs2rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        nb::arg("rpy"), "Convert ocs2-style roll-pitch-yaw angles to quaternion (returns xyzw vector)");
    rotations_module.def("rpy2mat", &tbai::rpy2mat, nb::arg("rpy"),
                         "Convert roll-pitch-yaw euler angles to rotation matrix");
    rotations_module.def("mat2aa", &tbai::mat2aa, nb::arg("mat"),
                         "Convert rotation matrix to axis-angle representation");

    // must be registered before any derived controllers
    // note: bind_g1 registers controllers too!
    nb::class_<tbai::Controller>(m, "Controller");

    // bind robots
    bind_go2(m);
    bind_go2_unitree(m);
    bind_g1(m);
    bind_g1_unitree(m);
    bind_franka(m);
    bind_anymal_b(m);
    bind_anymal_c(m);
    bind_anymal_d(m);
    bind_spot(m);

    // bind controllers
    bind_central_controller(m);
    bind_static_controller(m);
    bind_bob_controller(m);
    bind_np3o_controller(m);
    bind_wtw_controller(m);
    bind_arm_mpc_controller(m);
    bind_quadruped_mpc_controller(m);
    bind_python_controller(m);
}
