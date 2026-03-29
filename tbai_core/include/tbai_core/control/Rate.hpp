#pragma once

#include <chrono>
#include <thread>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>

namespace tbai {

/**
 * @brief Fixed-rate loop timer using a hybrid coarse-sleep + spin-wait strategy.
 *
 * Calling sleep() blocks until the next tick at the configured rate. For waits longer
 * than 2 ms, the thread yields to the OS scheduler and wakes ~1 ms early; the remaining
 * time is covered by a low-overhead spin-wait (x86 PAUSE / ARM YIELD). If the caller
 * falls more than one full interval behind, the timeline resets to avoid catch-up bursts.
 *
 * @tparam SCALAR_T  Arithmetic type for the rate (e.g. float or double).
 * @tparam CLOCK_T   Monotonic clock (default: std::chrono::high_resolution_clock).
 */
template <typename SCALAR_T, typename CLOCK_T = std::chrono::high_resolution_clock>
class SystemRate {
   public:
    SystemRate(SCALAR_T rate)
        : rate_(rate),
          interval_(std::chrono::duration_cast<typename CLOCK_T::duration>(
              std::chrono::duration<double>(1.0 / rate))),
          nextTime_(CLOCK_T::now()) {}

    SCALAR_T getRate() const { return rate_; }

    void sleep() {
        auto now = CLOCK_T::now();

        // Check if we are already behind schedule
        if (now >= nextTime_ + interval_) {
            nextTime_ = now;  // Reset to avoid "catch-up" bursts
            return;
        }

        // Target the next exact tick
        nextTime_ += interval_;

        // Coarse sleep: if the wait is long (> 2ms), give up the CPU slice.
        // Wake up slightly early (1ms buffer) to account for OS wakeup latency.
        if (nextTime_ - now > std::chrono::milliseconds(2)) {
            std::this_thread::sleep_until(nextTime_ - std::chrono::milliseconds(1));
        }

        // Precise spin: busy-wait for the final micro-moments
        while (CLOCK_T::now() < nextTime_) {
#if defined(__x86_64__) || defined(_M_X64)
            _mm_pause();
#elif defined(__arm__) || defined(__aarch64__)
            asm volatile("yield");
#endif
        }
    }

   private:
    SCALAR_T rate_;
    typename CLOCK_T::duration interval_;
    typename CLOCK_T::time_point nextTime_;
};

template <typename CLOCK_T = std::chrono::high_resolution_clock>
struct SystemTime {
    static inline scalar_t rightNow() { return convertToScalar(CLOCK_T::now()); }
};

}  // namespace tbai