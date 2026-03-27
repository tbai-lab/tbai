#pragma once

#include <chrono>
#include <functional>
#include <vector>

#include <tbai_core/Types.hpp>

namespace tbai {

/**
 * @brief Write the controller initialization timestamp to /tmp/tbai_init_time_123.
 *
 * This timestamp serves as a reference time for all controllers in the system.
 * Writing is both thread-safe and process-safe.
 *
 * The no-argument overload captures the current system clock time.
 */
void writeInitTime();

/**
 * @brief Read the controller initialization timestamp from /tmp/tbai_init_time_123.
 *
 * Reading is both thread-safe and process-safe.
 *
 * Throws if the file does not exist (i.e. writeInitTime was never called).
 * @return Seconds since epoch, as previously written by writeInitTime()
 */
scalar_t readInitTime();

/**
 * @brief Download a file from a Hugging Face repo, caching it under $TBAI_CACHE_DIR (default /tmp/tbai_hf_cache).
 * @note This function requires that `huggingface-cli` is installed, otherwise an error is thrown.
 * @param repo_id Hugging Face repository (e.g. "org/model-name")
 * @param filename File path within the repo; trailing '/' downloads a folder
 * @return Local filesystem path to the downloaded file
 */
std::string downloadFromHuggingFace(const std::string &repo_id, const std::string &filename);

/**
 * @overload Write init time from separate seconds and nanoseconds.
 * @param seconds Whole seconds since epoch
 * @param nanoseconds Fractional nanoseconds within the current second
 */
void writeInitTime(const long seconds, const long nanoseconds);

/**
 * @overload Write an already-computed init time (e.g. from ROS or a custom clock).
 * @param time Seconds since epoch
 */
void writeInitTime(const scalar_t time);

/**
 * @brief Stack vectors vertically
 * @param vectors The vectors to stack
 * @return The stacked vector
 */
tbai::vector_t vvstack(const std::vector<std::reference_wrapper<const tbai::vector_t>> &vectors);

template <typename TIMEPOINT>
inline scalar_t convertToScalar(const TIMEPOINT &time) {
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch() % std::chrono::seconds(1)).count();
    return static_cast<scalar_t>(seconds) + static_cast<scalar_t>(nanoseconds) * 1e-9;
}

}  // namespace tbai