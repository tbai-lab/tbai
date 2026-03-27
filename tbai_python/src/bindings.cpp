#include <cstring>
#include <iostream>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
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
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_deploy_g1/G1RLController.hpp>
#include <tbai_deploy_g1/G1MimicController.hpp>
#include <tbai_deploy_g1/G1ASAPController.hpp>
#include <tbai_deploy_g1/G1ASAPMimicController.hpp>
#include <tbai_deploy_g1/G1PBHCController.hpp>
#include <tbai_deploy_g1/G1Twist2Controller.hpp>
#include <tbai_deploy_g1/G1SpinkickController.hpp>
#include <tbai_deploy_g1/G1BeyondMimicController.hpp>
#endif

// Python wrappers around virtual classes
namespace tbai {

class PyRobotInterface : public tbai::RobotInterface {
   public:
    using RobotInterface::RobotInterface;
    void waitTillInitialized() override { PYBIND11_OVERRIDE_PURE(void, tbai::RobotInterface, waitTillInitialized); }
    State getLatestState() override { PYBIND11_OVERRIDE_PURE(State, tbai::RobotInterface, getLatestState); }
    void publish(std::vector<MotorCommand> commands) override {
        PYBIND11_OVERRIDE_PURE(void, tbai::RobotInterface, publish, commands);
    }
};

class PyChangeControllerSubscriber : public tbai::ChangeControllerSubscriber {
   public:
    using ChangeControllerSubscriber::ChangeControllerSubscriber;

    void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) override {
        PYBIND11_OVERRIDE(void, tbai::ChangeControllerSubscriber, setCallbackFunction, callbackFunction);
    }
    void triggerCallbacks() override {
        PYBIND11_OVERRIDE_PURE(void, tbai::ChangeControllerSubscriber, triggerCallbacks);
    }
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
            pybind11::gil_scoped_acquire acquire;
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
            pybind11::gil_scoped_acquire acquire;
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
            pybind11::gil_scoped_acquire acquire;
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
            pybind11::gil_scoped_acquire acquire;
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
    using reference::ReferenceVelocityGenerator::ReferenceVelocityGenerator;

    virtual tbai::reference::ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) override {
        PYBIND11_OVERRIDE_PURE(tbai::reference::ReferenceVelocity, tbai::reference::ReferenceVelocityGenerator,
                               getReferenceVelocity, time, dt);
    }
};

typedef tbai::CentralController<tbai::SystemRate<scalar_t>, tbai::SystemTime<std::chrono::high_resolution_clock>>
    CentralControllerPython;

}  // namespace tbai

namespace py = pybind11;

PYBIND11_MODULE(_C, m) {
    m.def("write_init_time", py::overload_cast<>(&tbai::writeInitTime));
    m.def("write_init_time", py::overload_cast<const long, const long>(&tbai::writeInitTime));
    m.def("write_init_time", py::overload_cast<const double>(&tbai::writeInitTime));
    m.def("read_init_time", &tbai::readInitTime);
    m.def("download_from_huggingface", &tbai::downloadFromHuggingFace, py::arg("repo_id"), py::arg("filename"),
          py::call_guard<py::gil_scoped_release>());

    py::class_<tbai::MotorCommand>(m, "MotorCommand")
        .def_readwrite("kp", &tbai::MotorCommand::kp)
        .def_readwrite("desired_position", &tbai::MotorCommand::desired_position)
        .def_readwrite("kd", &tbai::MotorCommand::kd)
        .def_readwrite("desired_velocity", &tbai::MotorCommand::desired_velocity)
        .def_readwrite("torque_ff", &tbai::MotorCommand::torque_ff)
        .def_readwrite("joint_name", &tbai::MotorCommand::joint_name);

    py::class_<tbai::State>(m, "State")
        .def(py::init<>())
        .def_readwrite("x", &tbai::State::x)
        .def_readwrite("timestamp", &tbai::State::timestamp)
        .def_readwrite("contact_flags", &tbai::State::contactFlags);

    py::class_<tbai::RobotInterface, tbai::PyRobotInterface, std::shared_ptr<tbai::RobotInterface>>(
        m, "RobotInterface")
        .def(py::init<>())
        .def("waitTillInitialized", &tbai::RobotInterface::waitTillInitialized)
        .def("getLatestState", &tbai::RobotInterface::getLatestState)
        .def("publish", &tbai::RobotInterface::publish);

    py::class_<tbai::ChangeControllerSubscriber, tbai::PyChangeControllerSubscriber,
               std::shared_ptr<tbai::ChangeControllerSubscriber>>(m, "ChangeControllerSubscriber")
        .def(py::init<>())
        .def("setCallbackFunction", &tbai::ChangeControllerSubscriber::setCallbackFunction)
        .def("triggerCallbacks", &tbai::ChangeControllerSubscriber::triggerCallbacks);

    py::class_<tbai::reference::ReferenceVelocity>(m, "ReferenceVelocity")
        .def(py::init<>())
        .def_readwrite("velocity_x", &tbai::reference::ReferenceVelocity::velocity_x)
        .def_readwrite("velocity_y", &tbai::reference::ReferenceVelocity::velocity_y)
        .def_readwrite("yaw_rate", &tbai::reference::ReferenceVelocity::yaw_rate);

    py::class_<tbai::reference::ReferenceVelocityGenerator, tbai::PyReferenceVelocityGenerator,
               std::shared_ptr<tbai::reference::ReferenceVelocityGenerator>>(m, "ReferenceVelocityGenerator")
        .def(py::init<>())
        .def("getReferenceVelocity", &tbai::reference::ReferenceVelocityGenerator::getReferenceVelocity);

    // Controller base class
    py::class_<tbai::Controller, std::shared_ptr<tbai::Controller>>(m, "Controller");

    // Concrete controller types
    py::class_<tbai::PyStaticController, tbai::Controller, std::shared_ptr<tbai::PyStaticController>>(
        m, "StaticController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             py::arg("robot_interface"), py::arg("post_step_callback") = nullptr);

    py::class_<tbai::PyNp3oController, tbai::Controller, std::shared_ptr<tbai::PyNp3oController>>(m, "Np3oController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             py::arg("robot_interface"), py::arg("ref_vel_gen"), py::arg("post_step_callback") = nullptr);

    py::class_<tbai::PyBobController, tbai::Controller, std::shared_ptr<tbai::PyBobController>>(m, "BobController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             py::arg("robot_interface"), py::arg("ref_vel_gen"), py::arg("post_step_callback") = nullptr);

    py::class_<tbai::PyWtwController, tbai::Controller, std::shared_ptr<tbai::PyWtwController>>(m, "WtwController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &,
                      std::function<void(tbai::scalar_t, tbai::scalar_t)>>(),
             py::arg("robot_interface"), py::arg("ref_vel_gen"), py::arg("post_step_callback") = nullptr);

    // Central controller with generic add_controller
    py::class_<tbai::CentralControllerPython, std::shared_ptr<tbai::CentralControllerPython>>(m, "CentralController")
        .def_static("create",
                    [](std::shared_ptr<tbai::RobotInterface> robotInterfacePtr,
                       std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriberPtr) {
                        return std::make_shared<tbai::CentralControllerPython>(robotInterfacePtr,
                                                                               changeControllerSubscriberPtr);
                    })
        .def("start", &tbai::CentralControllerPython::start, py::call_guard<py::gil_scoped_release>())
        .def("startThread", &tbai::CentralControllerPython::startThread, py::call_guard<py::gil_scoped_release>())
        .def("stopThread", &tbai::CentralControllerPython::stopThread, py::call_guard<py::gil_scoped_release>())
        .def("add_controller", &tbai::CentralControllerPython::addController, py::arg("controller"),
             py::arg("make_active") = false)
        .def("initialize", &tbai::CentralControllerPython::initialize, py::call_guard<py::gil_scoped_release>())
        .def("step", &tbai::CentralControllerPython::step, py::call_guard<py::gil_scoped_release>())
        .def("getRate", &tbai::CentralControllerPython::getRate);

    // Bind rotation helper functions
    py::module rotations_module = m.def_submodule("rotations");
    rotations_module.def(
        "rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        "Convert roll-pitch-yaw euler angles to quaternion (returns xyzw vector)");
    rotations_module.def(
        "quat2mat",
        [](const tbai::vector4_t &q) {
            tbai::matrix3_t mat = tbai::quat2mat(tbai::quaternion_t(q[3], q[0], q[1], q[2]));
            return tbai::matrix3_t(mat);
        },
        "Convert quaternion to rotation matrix, expects xyzw vector");
    rotations_module.def("mat2rpy", &tbai::mat2rpy, "Convert rotation matrix to roll-pitch-yaw euler angles");
    rotations_module.def("mat2ocs2rpy", &tbai::mat2oc2rpy,
                         "Convert rotation matrix to ocs2-style roll-pitch-yaw euler angles");
    rotations_module.def(
        "ocs2rpy2quat",
        [](const tbai::vector3_t &rpy) {
            tbai::quaternion_t q = tbai::ocs2rpy2quat(rpy);
            return tbai::vector4_t(q.x(), q.y(), q.z(), q.w());
        },
        "Convert ocs2-style roll-pitch-yaw angles to quaternion (returns xyzw vector)");
    rotations_module.def("rpy2mat", &tbai::rpy2mat, "Convert roll-pitch-yaw euler angles to rotation matrix");
    rotations_module.def("mat2aa", &tbai::mat2aa, "Convert rotation matrix to axis-angle representation");


#ifdef TBAI_HAS_DEPLOY_GO2
    py::class_<tbai::Go2RobotInterfaceArgs>(m, "Go2RobotInterfaceArgs")
        .def(py::init<>())
        .def(
            "set_network_interface",
            [](tbai::Go2RobotInterfaceArgs &self, const std::string &val) -> tbai::Go2RobotInterfaceArgs & {
                self.networkInterface(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_unitree_channel",
            [](tbai::Go2RobotInterfaceArgs &self, int val) -> tbai::Go2RobotInterfaceArgs & {
                self.unitreeChannel(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_channel_init",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.channelInit(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_enable_state_estim",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.enableStateEstim(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_subscribe_lidar",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.subscribeLidar(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_enable_video",
            [](tbai::Go2RobotInterfaceArgs &self, bool val) -> tbai::Go2RobotInterfaceArgs & {
                self.enableVideo(val);
                return self;
            },
            py::return_value_policy::reference_internal);

    py::class_<tbai::Go2RobotInterface, tbai::RobotInterface, std::shared_ptr<tbai::Go2RobotInterface>>(
        m, "Go2RobotInterface")
        .def(py::init<tbai::Go2RobotInterfaceArgs>())
        .def("publish", &tbai::Go2RobotInterface::publish, py::call_guard<py::gil_scoped_release>())
        .def("waitTillInitialized", &tbai::Go2RobotInterface::waitTillInitialized,
             py::call_guard<py::gil_scoped_release>())
        .def("getLatestState", &tbai::Go2RobotInterface::getLatestState,
             py::call_guard<py::gil_scoped_release>());
#endif

    m.attr("HAS_DEPLOY_GO2") =
#ifdef TBAI_HAS_DEPLOY_GO2
        true;
#else
        false;
#endif

#ifdef TBAI_HAS_DEPLOY_G1
    py::class_<tbai::G1RobotInterfaceArgs>(m, "G1RobotInterfaceArgs")
        .def(py::init<>())
        .def(
            "set_network_interface",
            [](tbai::G1RobotInterfaceArgs &self, const std::string &val) -> tbai::G1RobotInterfaceArgs & {
                self.networkInterface(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_unitree_channel",
            [](tbai::G1RobotInterfaceArgs &self, int val) -> tbai::G1RobotInterfaceArgs & {
                self.unitreeChannel(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_channel_init",
            [](tbai::G1RobotInterfaceArgs &self, bool val) -> tbai::G1RobotInterfaceArgs & {
                self.channelInit(val);
                return self;
            },
            py::return_value_policy::reference_internal)
        .def(
            "set_enable_state_estim",
            [](tbai::G1RobotInterfaceArgs &self, bool val) -> tbai::G1RobotInterfaceArgs & {
                self.enableStateEstim(val);
                return self;
            },
            py::return_value_policy::reference_internal);

    py::class_<tbai::G1RobotInterface, tbai::RobotInterface, std::shared_ptr<tbai::G1RobotInterface>>(
        m, "G1RobotInterface")
        .def(py::init<tbai::G1RobotInterfaceArgs>())
        .def("publish", &tbai::G1RobotInterface::publish, py::call_guard<py::gil_scoped_release>())
        .def("waitTillInitialized", &tbai::G1RobotInterface::waitTillInitialized,
             py::call_guard<py::gil_scoped_release>())
        .def("getLatestState", &tbai::G1RobotInterface::getLatestState,
             py::call_guard<py::gil_scoped_release>())
        .def("getBaseQuaternion", &tbai::G1RobotInterface::getBaseQuaternion);

    py::class_<tbai::g1::G1RLController, tbai::Controller, std::shared_ptr<tbai::g1::G1RLController>>(
        m, "G1RLController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &>(),
             py::arg("robot_interface"), py::arg("ref_vel_gen"), py::arg("policy_path"));

    py::class_<tbai::g1::G1MimicController, tbai::Controller, std::shared_ptr<tbai::g1::G1MimicController>>(
        m, "G1MimicController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, float, const std::string &>(),
             py::arg("robot_interface"), py::arg("policy_path"), py::arg("motion_file_path"),
             py::arg("motion_fps") = 60.0f, py::arg("time_start") = 0.0f, py::arg("time_end") = -1.0f,
             py::arg("controller_name") = "G1MimicController")
        .def("isMotionComplete", &tbai::g1::G1MimicController::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1MimicController::getMotionTime);

    py::class_<tbai::g1::G1ASAPController, tbai::Controller, std::shared_ptr<tbai::g1::G1ASAPController>>(
        m, "G1ASAPController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &, const std::string &,
                      const std::string &>(),
             py::arg("robot_interface"), py::arg("ref_vel_gen"), py::arg("policy_path"),
             py::arg("controller_name") = "G1ASAPLocomotion");

    py::class_<tbai::g1::G1ASAPMimicController, tbai::Controller, std::shared_ptr<tbai::g1::G1ASAPMimicController>>(
        m, "G1ASAPMimicController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, float,
                      const std::string &>(),
             py::arg("robot_interface"), py::arg("policy_path"), py::arg("motion_length"),
             py::arg("controller_name") = "G1ASAPMimic")
        .def("isMotionComplete", &tbai::g1::G1ASAPMimicController::isMotionComplete)
        .def("getMotionPhase", &tbai::g1::G1ASAPMimicController::getMotionPhase);

    py::class_<tbai::g1::G1PBHCController, tbai::Controller, std::shared_ptr<tbai::g1::G1PBHCController>>(
        m, "G1PBHCController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             py::arg("robot_interface"), py::arg("policy_path"), py::arg("motion_file_path"),
             py::arg("time_start") = 0.0f, py::arg("time_end") = -1.0f,
             py::arg("controller_name") = "G1PBHCController")
        .def("isMotionComplete", &tbai::g1::G1PBHCController::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1PBHCController::getMotionTime);

    py::class_<tbai::g1::G1Twist2Controller, tbai::Controller, std::shared_ptr<tbai::g1::G1Twist2Controller>>(
        m, "G1Twist2Controller")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, float,
                      float, const std::string &>(),
             py::arg("robot_interface"), py::arg("policy_path"), py::arg("motion_file_path"),
             py::arg("time_start") = 0.0f, py::arg("time_end") = -1.0f,
             py::arg("controller_name") = "G1Twist2Controller")
        .def("isMotionComplete", &tbai::g1::G1Twist2Controller::isMotionComplete)
        .def("getMotionTime", &tbai::g1::G1Twist2Controller::getMotionTime);

    py::class_<tbai::g1::G1SpinkickController, tbai::Controller, std::shared_ptr<tbai::g1::G1SpinkickController>>(
        m, "G1SpinkickController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             py::arg("robot_interface"), py::arg("policy_path"),
             py::arg("controller_name") = "G1SpinkickController", py::arg("use_model_meta_config") = true,
             py::arg("action_beta") = 1.0f)
        .def("isMotionComplete", &tbai::g1::G1SpinkickController::isMotionComplete)
        .def("getTimestep", &tbai::g1::G1SpinkickController::getTimestep)
        .def("getMaxTimestep", &tbai::g1::G1SpinkickController::getMaxTimestep);

    py::class_<tbai::g1::G1BeyondMimicController, tbai::Controller,
               std::shared_ptr<tbai::g1::G1BeyondMimicController>>(m, "G1BeyondMimicController")
        .def(py::init<const std::shared_ptr<tbai::RobotInterface> &, const std::string &, const std::string &, bool,
                      float>(),
             py::arg("robot_interface"), py::arg("policy_path"),
             py::arg("controller_name") = "G1BeyondMimicController", py::arg("use_model_meta_config") = true,
             py::arg("action_beta") = 1.0f)
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
