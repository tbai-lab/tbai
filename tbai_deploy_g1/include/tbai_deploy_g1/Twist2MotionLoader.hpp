#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief TWIST2 Motion Loader
 *
 * Computes velocities and provides action_mimic (35 dims):
 * - xy_vel: 2 dims (local frame)
 * - z_pos: 1 dim
 * - roll, pitch: 2 dims
 * - yaw_ang_vel: 1 dim (local frame)
 * - dof_pos: 29 dims
 */
class Twist2MotionLoader {
   public:
    /**
     * @brief Construct a new Twist2MotionLoader
     * @param motionFilePath Path to motion CSV file
     */
    explicit Twist2MotionLoader(const std::string &motionFilePath);

    ~Twist2MotionLoader() = default;

    /**
     * @brief Get the current action_mimic observation (35 dims)
     * @return action_mimic: xy_vel(2) + z(1) + roll(1) + pitch(1) + yaw_ang_vel(1) + dof_pos(29)
     */
    vector_t getActionMimic() const;

    /**
     * @brief Get current root position (3 dims)
     */
    vector_t getRootPosition() const;

    /**
     * @brief Get current root quaternion (4 dims: x,y,z,w)
     */
    vector_t getRootQuaternion() const;

    /**
     * @brief Get current root linear velocity (3 dims)
     */
    vector_t getRootVelocity() const;

    /**
     * @brief Get current root angular velocity (3 dims)
     */
    vector_t getRootAngularVelocity() const;

    /**
     * @brief Get current DOF positions (29 dims)
     */
    vector_t getDofPositions() const;

    /**
     * @brief Advance to next frame based on time
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

    /**
     * @brief Rotate vector from world to local frame using quaternion
     */
    vector_t rotateToLocal(const vector_t &vec, const vector_t &quat) const;

    /**
     * @brief Convert quaternion to euler angles (roll, pitch, yaw)
     */
    vector_t quaternionToEuler(const vector_t &quat) const;

    // Motion data
    int numFrames_;
    float fps_;
    float dt_;

    // Raw data (column-major Eigen matrices)
    Eigen::MatrixXd rootPos_;      // (3, num_frames)
    Eigen::MatrixXd rootRot_;      // (4, num_frames) quaternion x,y,z,w
    Eigen::MatrixXd dofPos_;       // (29, num_frames)

    // Computed velocities
    Eigen::MatrixXd rootVel_;      // (3, num_frames)
    Eigen::MatrixXd rootAngVel_;   // (3, num_frames)

    // Current interpolated state
    vector_t currentRootPos_;
    vector_t currentRootRot_;
    vector_t currentRootVel_;
    vector_t currentRootAngVel_;
    vector_t currentDofPos_;

    // Playback state
    float currentTime_;
    float timeStart_;
    float timeEnd_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
