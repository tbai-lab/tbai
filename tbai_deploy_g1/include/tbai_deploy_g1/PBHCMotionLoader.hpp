#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief PBHC Motion Loader for loading pkl motion files.
 *
 * Loads motion data from PBHC pkl format (via CSV conversion):
 * - fps: frames per second (typically 50 Hz)
 * - root_pos: (num_frames, 3) root position
 * - root_rot: (num_frames, 4) root quaternion (x,y,z,w)
 * - dof_pos: (num_frames, 23) joint positions (23 DOF locked wrists)
 * - dof_vel: (num_frames, 23) joint velocities
 *
 * Provides:
 * - Motion interpolation at any time
 * - ref_motion_phase calculation (0 to 1 over motion duration)
 * - DOF positions for initial state
 */
class PBHCMotionLoader {
   public:
    /**
     * @brief Construct a new PBHCMotionLoader
     * @param motionFilePath Path to motion CSV file (converted from pkl)
     */
    explicit PBHCMotionLoader(const std::string &motionFilePath);

    ~PBHCMotionLoader() = default;

    /**
     * @brief Get current ref_motion_phase (0 to 1)
     */
    scalar_t getRefMotionPhase() const;

    /**
     * @brief Get current DOF positions (23 dims)
     */
    vector_t getDofPositions() const;

    /**
     * @brief Get current DOF velocities (23 dims)
     */
    vector_t getDofVelocities() const;

    /**
     * @brief Get current root position (3 dims)
     */
    vector_t getRootPosition() const;

    /**
     * @brief Get current root quaternion (4 dims: x,y,z,w)
     */
    vector_t getRootQuaternion() const;

    /**
     * @brief Advance motion time
     * @param dt Time step
     * @return true if motion still active, false if completed
     */
    bool advance(float dt);

    /**
     * @brief Reset to start of motion
     */
    void reset();

    /**
     * @brief Set time range for playback
     * @param timeStart Start time in seconds
     * @param timeEnd End time in seconds (-1 for full duration)
     */
    void setTimeRange(float timeStart, float timeEnd);

    /**
     * @brief Get motion duration in seconds
     */
    float getDuration() const;

    /**
     * @brief Get current time in motion
     */
    float getCurrentTime() const { return currentTime_; }

    /**
     * @brief Check if motion is complete
     */
    bool isComplete() const { return currentTime_ >= timeEnd_; }

    /**
     * @brief Get number of frames
     */
    int getNumFrames() const { return numFrames_; }

    /**
     * @brief Get frames per second
     */
    float getFps() const { return fps_; }

   private:
    void loadCsvFile(const std::string &motionFilePath);
    void interpolateFrame(float time);

    // Motion data
    int numFrames_;
    float fps_;
    float dt_;

    // Raw data (column-major Eigen matrices)
    Eigen::MatrixXd rootPos_;    // (3, num_frames)
    Eigen::MatrixXd rootRot_;    // (4, num_frames) quaternion x,y,z,w
    Eigen::MatrixXd dofPos_;     // (23, num_frames)
    Eigen::MatrixXd dofVel_;     // (23, num_frames)

    // Current interpolated state
    vector_t currentRootPos_;
    vector_t currentRootRot_;
    vector_t currentDofPos_;
    vector_t currentDofVel_;

    // Playback state
    float currentTime_;
    float timeStart_;
    float timeEnd_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
