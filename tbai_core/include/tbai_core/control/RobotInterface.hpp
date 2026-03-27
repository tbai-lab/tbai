#pragma once

#include <functional>
#include <string>
#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

struct State {
    vector_t x;
    scalar_t timestamp;
    std::vector<bool> contactFlags;
};

struct MotorCommand {
    scalar_t kp;
    scalar_t desired_position;
    scalar_t kd;
    scalar_t desired_velocity;
    scalar_t torque_ff;
    std::string joint_name;
};

class RobotInterface {
   public:
    virtual ~RobotInterface() = default;

    virtual void waitTillInitialized() = 0;
    virtual State getLatestState() = 0;

    virtual void enableEstimator() {}
    virtual void disableEstimator() {}

    virtual void publish(std::vector<MotorCommand> commands) = 0;
};

class ChangeControllerSubscriber {
   public:
    virtual ~ChangeControllerSubscriber() = default;

    virtual void setCallbackFunction(std::function<void(const std::string &controllerType)> callbackFunction) {
        callbackFunction_ = callbackFunction;
    }

    virtual void triggerCallbacks() = 0;

   protected:
    std::function<void(const std::string &controllerType)> callbackFunction_;
};

}  // namespace tbai
