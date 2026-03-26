#pragma once

#include <stdint.h>

#include <cmath>
#include <mutex>
#include <atomic>

#include <tbai_core/Logging.hpp>

namespace tbai {
namespace go2 {

class Go2JoystickInterface {
   public:
    Go2JoystickInterface();
    virtual ~Go2JoystickInterface() = default;

    virtual void onPressA() { TBAI_LOG_WARN(logger_, "A pressed"); }
    virtual void onPressB() { TBAI_LOG_WARN(logger_, "B pressed"); }
    virtual void onPressX() { TBAI_LOG_WARN(logger_, "X pressed"); }
    virtual void onPressY() { TBAI_LOG_WARN(logger_, "Y pressed"); }
    virtual void onReleaseA() { TBAI_LOG_WARN(logger_, "A released"); }
    virtual void onReleaseB() { TBAI_LOG_WARN(logger_, "B released"); }
    virtual void onReleaseX() { TBAI_LOG_WARN(logger_, "X released"); }
    virtual void onReleaseY() { TBAI_LOG_WARN(logger_, "Y released"); }
    virtual void onPressStart() { TBAI_LOG_WARN(logger_, "Start pressed"); }
    virtual void onPressSelect() { TBAI_LOG_WARN(logger_, "Select pressed"); }
    virtual void onPressL1() { TBAI_LOG_WARN(logger_, "L1 pressed"); }
    virtual void onPressR1() { TBAI_LOG_WARN(logger_, "R1 pressed"); }
    virtual void onPressL2() { TBAI_LOG_WARN(logger_, "L2 pressed"); }
    virtual void onPressR2() { TBAI_LOG_WARN(logger_, "R2 pressed"); }
    virtual void onPressF1() { TBAI_LOG_WARN(logger_, "F1 pressed"); }
    virtual void onPressUp() { TBAI_LOG_WARN(logger_, "Up pressed"); }
    virtual void onPressRight() { TBAI_LOG_WARN(logger_, "Right pressed"); }
    virtual void onPressDown() { TBAI_LOG_WARN(logger_, "Down pressed"); }
    virtual void onPressLeft() { TBAI_LOG_WARN(logger_, "Left pressed"); }
    virtual void onReleaseUp() { TBAI_LOG_WARN(logger_, "Up released"); }
    virtual void onReleaseRight() { TBAI_LOG_WARN(logger_, "Right released"); }
    virtual void onReleaseDown() { TBAI_LOG_WARN(logger_, "Down released"); }
    virtual void onReleaseLeft() { TBAI_LOG_WARN(logger_, "Left released"); }
    virtual void Start() {}

   protected:
    float lx = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    float ly = 0.0f;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace go2
}  // namespace tbai
