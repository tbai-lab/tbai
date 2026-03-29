#include <cstring>
#include <iostream>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
#include <tbai_bob/BobController.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Rate.hpp>
#include <tbai_core/control/RobotInterface.hpp>
#include <tbai_np3o/Np3oController.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_static/StaticController.hpp>
#include <tbai_wtw/WtwController.hpp>

#ifdef TBAI_HAS_DEPLOY_GO2
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#endif

#ifdef TBAI_HAS_DEPLOY_G1
#include <tbai_deploy_g1/G1ASAPController.hpp>
#include <tbai_deploy_g1/G1ASAPMimicController.hpp>
#include <tbai_deploy_g1/G1BeyondMimicController.hpp>
#include <tbai_deploy_g1/G1MimicController.hpp>
#include <tbai_deploy_g1/G1PBHCController.hpp>
#include <tbai_deploy_g1/G1RLController.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_deploy_g1/G1SpinkickController.hpp>
#include <tbai_deploy_g1/G1Twist2Controller.hpp>
#endif

// Python wrappers around virtual classes
namespace tbai {

class PyRobotInterface : public tbai::RobotInterface {
   public:
    NB_TRAMPOLINE(tbai::RobotInterface, 3);
    void waitTillInitialized() override { NB_OVERRIDE_PURE(waitTillInitialized); }
    State getLatestState() override { NB_OVERRIDE_PURE(getLatestState); }
    void publish(std::vector<MotorCommand> commands) override { NB_OVERRIDE_PURE(publish, commands); }
};

class PyChangeControllerSubscriber : public tbai::ChangeControllerSubscriber {
   public:
    NB_TRAMPOLINE(tbai::ChangeControllerSubscriber, 2);

    void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) override {
        NB_OVERRIDE(setCallbackFunction, callbackFunction);
    }
    void triggerCallbacks() override { NB_OVERRIDE_PURE(triggerCallbacks); }
};

class PyStaticController : public tbai::static_::StaticController {
   public:
    PyStaticController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                       std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::static_::StaticController(robotInterfacePtr), postStepCallback_(postStepCallback) {}

    bool ok() const override { return true; }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = robotInterfacePtr_->getLatestState(); }

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

class PyBobController : public tbai::BobController {
   public:
    PyBobController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen,
                    std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::BobController(robotInterfacePtr, refVelGen), postStepCallback_(postStepCallback) {}

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {
        // Do nothing
    }

    void atPositions(matrix_t &positions) override {
        // Do nothing
    }

    bool ok() const override { return true; }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

class PyNp3oController : public tbai::Np3oController {
   public:
    PyNp3oController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen,
                     std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::Np3oController(robotInterfacePtr, refVelGen), postStepCallback_(postStepCallback) {}

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {
        // Do nothing
    }

    bool ok() const override { return true; }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

class PyWtwController : public tbai::WtwController {
   public:
    PyWtwController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGen,
                    std::function<void(scalar_t, scalar_t)> postStepCallback = nullptr)
        : tbai::WtwController(robotInterfacePtr, refVelGen), postStepCallback_(postStepCallback) {}

    void postStep(scalar_t currentTime, scalar_t dt) override {
        if (postStepCallback_) {
            nanobind::gil_scoped_acquire acquire;
            postStepCallback_(currentTime, dt);
        }
    }

    void waitTillInitialized() override { robotInterfacePtr_->waitTillInitialized(); }

    void changeController(const std::string &controllerType, scalar_t currentTime) override {
        preStep(currentTime, 0.0);
    }

    bool ok() const override { return true; }

    void preStep(scalar_t currentTime, scalar_t dt) override { state_ = robotInterfacePtr_->getLatestState(); }

    std::function<void(scalar_t, scalar_t)> postStepCallback_ = nullptr;
};

class PyReferenceVelocityGenerator : public tbai::reference::ReferenceVelocityGenerator {
   public:
    NB_TRAMPOLINE(tbai::reference::ReferenceVelocityGenerator, 1);

    tbai::reference::ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override {
        NB_OVERRIDE_PURE(getReferenceVelocity, time, dt);
    }
};

typedef tbai::CentralController<tbai::SystemRate<scalar_t>, tbai::SystemTime<std::chrono::high_resolution_clock>>
    CentralControllerPython;

}  // namespace tbai

namespace nb = nanobind;

NB_MODULE(_C, m) {
    nb::set_leak_warnings(false);

    m.def("write_init_time", nb::overload_cast<>(&tbai::writeInitTime));
    m.def("write_init_time", nb::overload_cast<const long, const long>(&tbai::writeInitTime));
    m.def("write_init_time", nb::overload_cast<const double>(&tbai::writeInitTime));
    m.def("read_init_time", &tbai::readInitTime);
    m.def("download_from_huggingface", &tbai::downloadFromHuggingFace, nb::arg("repo_id"), nb::arg("filename"),
          nb::call_guard<nb::gil_scoped_release>());

    nb::class_<tbai::MotorCommand>(m, "MotorCommand")
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
        .def("waitTillInitialized", &tbai::RobotInterface::waitTillInitialized)
        .def("getLatestState", &tbai::RobotInterface::getLatestState)
        .def("publish", &tbai::RobotInterface::publish);

    nb::class_<tbai::ChangeControllerSubscriber, tbai::PyChangeControllerSubscriber>(m, "ChangeControllerSubscriber")
        .def(nb::init<>())
        .def("setCallbackFunction", &tbai::ChangeControllerSubscriber::setCallbackFunction)
        .def("triggerCallbacks", &tbai::ChangeControllerSubscriber::triggerCallbacks);

    nb::class_<tbai::reference::ReferenceVelocity>(m, "ReferenceVelocity")
        .def(nb::init<>())
        .def_rw("velocity_x", &tbai::reference::ReferenceVelocity::velocity_x)
        .def_rw("velocity_y", &tbai::reference::ReferenceVelocity::velocity_y)
        .def_rw("yaw_rate", &tbai::reference::ReferenceVelocity::yaw_rate);

    nb::class_<tbai::reference::ReferenceVelocityGenerator, tbai::PyReferenceVelocityGenerator>(m, "ReferenceVelocityGenerator")
        .def(nb::init<>())
        .def("getReferenceVelocity", &tbai::reference::ReferenceVelocityGenerator::getReferenceVelocity);

    // Controller base class
    nb::class_<tbai::Controller>(m, "Controller");

    // Concrete controller types
    nb::class_<tbai::PyStaticController, tbai::Controller>(
        m, "StaticController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("post_step_callback") = nullptr);

    nb::class_<tbai::PyNp3oController, tbai::Controller>(m, "Np3oController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("post_step_callback") = nullptr);

    nb::class_<tbai::PyBobController, tbai::Controller>(m, "BobController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("post_step_callback") = nullptr);

    nb::class_<tbai::PyWtwController, tbai::Controller>(m, "WtwController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("post_step_callback") = nullptr);

    // Central controller with generic add_controller
    nb::class_<tbai::CentralControllerPython>(m, "CentralController")
        .def_static("create",
                    [](std::shared_ptr<tbai::RobotInterface> robotInterfacePtr,
                       std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriberPtr) {
                        return std::make_shared<tbai::CentralControllerPython>(robotInterfacePtr,
                                                                               changeControllerSubscriberPtr);
                    })
        .def("start", &tbai::CentralControllerPython::start, nb::call_guard<nb::gil_scoped_release>())
        .def("startThread", &tbai::CentralControllerPython::startThread, nb::call_guard<nb::gil_scoped_release>())
        .def("stopThread", &tbai::CentralControllerPython::stopThread, nb::call_guard<nb::gil_scoped_release>())
        .def("add_controller", &tbai::CentralControllerPython::addController, nb::arg("controller"),
             nb::arg("make_active") = false)
        .def("initialize", &tbai::CentralControllerPython::initialize, nb::call_guard<nb::gil_scoped_release>())
        .def("step", &tbai::CentralControllerPython::step, nb::call_guard<nb::gil_scoped_release>())
        .def("getRate", &tbai::CentralControllerPython::getRate);

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
            tbai::matrix3_t mat = tbai::quat2mat(tbai::quaternion_t(q[3], q[0], q[1], q[2])); // Eigen quaternion expects wxyz order
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

#ifdef TBAI_HAS_DEPLOY_GO2
    nb::class_<tbai::Go2RobotInterfaceArgs>(m, "Go2RobotInterfaceArgs")
        .def(nb::init<>())
        .def(
            "set_network_interface",
            [](tbai::Go2RobotInterfaceArgs &self, const std::string &val) -> tbai::Go2RobotInterfaceArgs & {
                self.networkInterface(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_unitree_channel",
            [](tbai::Go2RobotInterfaceArgs &self, int val) -> tbai::Go2RobotInterfaceArgs & {
                self.unitreeChannel(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_channel_init",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.channelInit(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_enable_state_estim",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.enableStateEstim(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_subscribe_lidar",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.subscribeLidar(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_enable_video",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.enableVideo(val);
                return self;
            },
            nb::rv_policy::reference_internal);

    nb::class_<tbai::Go2RobotInterface, tbai::RobotInterface>(m, "Go2RobotInterface")
        .def(nb::init<tbai::Go2RobotInterfaceArgs>())
        .def("publish", &tbai::Go2RobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("waitTillInitialized", &tbai::Go2RobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("getLatestState", &tbai::Go2RobotInterface::getLatestState, nb::call_guard<nb::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_GO2") =
#ifdef TBAI_HAS_DEPLOY_GO2
        true;
#else
        false;
#endif

#ifdef TBAI_HAS_DEPLOY_G1
    nb::class_<tbai::G1RobotInterfaceArgs>(m, "G1RobotInterfaceArgs")
        .def(nb::init<>())
        .def(
            "set_network_interface",
            [](tbai::G1RobotInterfaceArgs &self, const std::string &val) -> tbai::G1RobotInterfaceArgs & {
                self.networkInterface(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_unitree_channel",
            [](tbai::G1RobotInterfaceArgs &self, int val) -> tbai::G1RobotInterfaceArgs & {
                self.unitreeChannel(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_channel_init",
            [](tbai::G1RobotInterfaceArgs &self, bool val) -> tbai::G1RobotInterfaceArgs & {
                self.channelInit(val);
                return self;
            },
            nb::rv_policy::reference_internal)
        .def(
            "set_enable_state_estim",
            [](tbai::G1RobotInterfaceArgs &self, bool val) -> tbai::G1RobotInterfaceArgs & {
                self.enableStateEstim(val);
                return self;
            },
            nb::rv_policy::reference_internal);

    nb::class_<tbai::G1RobotInterface, tbai::RobotInterface>(m, "G1RobotInterface")
        .def(nb::init<tbai::G1RobotInterfaceArgs>())
        .def("publish", &tbai::G1RobotInterface::publish, nb::call_guard<nb::gil_scoped_release>())
        .def("waitTillInitialized", &tbai::G1RobotInterface::waitTillInitialized,
             nb::call_guard<nb::gil_scoped_release>())
        .def("getLatestState", &tbai::G1RobotInterface::getLatestState, nb::call_guard<nb::gil_scoped_release>())
        .def("getBaseQuaternion", &tbai::G1RobotInterface::getBaseQuaternion);

    nb::class_<tbai::g1::G1RLController, tbai::Controller>(m, "G1RLController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("policy_path"));

    nb::class_<tbai::g1::G1MimicController, tbai::Controller>(m, "G1MimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("motion_fps") = 60.0f, nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f,
             nb::arg("controller_name") = "G1MimicController")
        .def("isMotionComplete", &tbai::g1::G1MimicController::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1MimicController::getMotionTime);

    nb::class_<tbai::g1::G1ASAPController, tbai::Controller>(m, "G1ASAPController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &,
                      const std::string &>(),
             nb::arg("robot_interface"), nb::arg("ref_vel_gen"), nb::arg("policy_path"),
             nb::arg("controller_name") = "G1ASAPLocomotion");

    nb::class_<tbai::g1::G1ASAPMimicController, tbai::Controller>(m, "G1ASAPMimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_length"),
             nb::arg("controller_name") = "G1ASAPMimic")
        .def("isMotionComplete", &tbai::g1::G1ASAPMimicController::isMotionComplete)
        .def("getMotionPhase", &tbai::g1::G1ASAPMimicController::getMotionPhase);

    nb::class_<tbai::g1::G1PBHCController, tbai::Controller>(m, "G1PBHCController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f, nb::arg("controller_name") = "G1PBHCController")
        .def("isMotionComplete", &tbai::g1::G1PBHCController::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1PBHCController::getMotionTime);

    nb::class_<tbai::g1::G1Twist2Controller, tbai::Controller>(m, "G1Twist2Controller")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("motion_file_path"),
             nb::arg("time_start") = 0.0f, nb::arg("time_end") = -1.0f,
             nb::arg("controller_name") = "G1Twist2Controller")
        .def("isMotionComplete", &tbai::g1::G1Twist2Controller::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1Twist2Controller::getMotionTime);

    nb::class_<tbai::g1::G1SpinkickController, tbai::Controller>(m, "G1SpinkickController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("controller_name") = "G1SpinkickController",
             nb::arg("use_model_meta_config") = true, nb::arg("action_beta") = 1.0f)
        .def("isMotionComplete", &tbai::g1::G1SpinkickController::isMotionComplete)
        .def("getTimestep", &tbai::g1::G1SpinkickController::getTimestep)
        .def("getMaxTimestep", &tbai::g1::G1SpinkickController::getMaxTimestep);

    nb::class_<tbai::g1::G1BeyondMimicController, tbai::Controller>(m, "G1BeyondMimicController")
        .def(nb::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             nb::arg("robot_interface"), nb::arg("policy_path"), nb::arg("controller_name") = "G1BeyondMimicController",
             nb::arg("use_model_meta_config") = true, nb::arg("action_beta") = 1.0f)
        .def("isMotionComplete", &tbai::g1::G1BeyondMimicController::isMotionComplete)
        .def("getTimestep", &tbai::g1::G1BeyondMimicController::getTimestep)
        .def("getMaxTimestep", &tbai::g1::G1BeyondMimicController::getMaxTimestep);
#endif

    m.attr("HAS_DEPLOY_G1") =
#ifdef TBAI_HAS_DEPLOY_G1
        true;
#else
        false;
#endif
}
