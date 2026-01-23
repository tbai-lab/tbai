#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tbai_core/Types.hpp>
#include <tbai_deploy_g1/MotionLoader.hpp>

namespace tbai {
namespace g1 {

// TWIST action_mimic constants
constexpr int TWIST_ACTION_MIMIC_DOF = 23;  // 23 DOF positions in action_mimic (excludes all wrist joints)
constexpr int TWIST_ACTION_MIMIC_SIZE = 31; // height(1) + RPY(3) + lin_vel(3) + yaw_ang_vel(1) + dof_pos(23) = 31

/**
 * @brief Extended MotionLoader that computes action_mimic for TWIST controller.
 *
 * The action_mimic vector (31 dims) contains:
 * - height (1): z-position of root
 * - RPY (3): roll, pitch, yaw of root
 * - lin_vel (3): linear velocity in local frame
 * - yaw_ang_vel (1): yaw angular velocity
 * - dof_pos (23): 23 TWIST joint positions (excludes wrist joints)
 */
class TwistMotionLoader : public MotionLoader {
   public:
    /**
     * @brief Construct a new TwistMotionLoader
     * @param motionFile Path to motion CSV file (29-DOF G1 joints)
     * @param fps Motion frames per second
     * @param twistJointIds Mapping from 23 TWIST joints to 29 G1 joints
     */
    TwistMotionLoader(const std::string &motionFile, float fps, const std::vector<int> &twistJointIds);

    /**
     * @brief Get action_mimic vector (31 dims) for TWIST controller.
     *
     * Format: [height(1), RPY(3), lin_vel(3), yaw_ang_vel(1), dof_pos(23)]
     * Note: dof_pos excludes all 6 wrist joints (uses 23 TWIST joints only)
     */
    vector_t getActionMimic() const;

    /**
     * @brief Get 23-DOF joint positions (TWIST order, no wrists)
     */
    vector_t jointPosTwist() const;

    /**
     * @brief Get 23-DOF joint velocities (TWIST order, no wrists)
     */
    vector_t jointVelTwist() const;

    /**
     * @brief Get root linear velocity in local frame
     */
    vector3_t rootLinearVelocity() const;

    /**
     * @brief Get root angular velocity
     */
    vector3_t rootAngularVelocity() const;

    /**
     * @brief Get root height (z-position)
     */
    scalar_t rootHeight() const;

    /**
     * @brief Get root RPY angles
     */
    vector3_t rootRPY() const;

   private:
    void computeVelocities();

    std::vector<int> twistJointIds_;  // Maps 23 TWIST joints to 29 G1 joints

    // Motion data with velocities
    std::vector<vector3_t> rootLinearVelocities_;
    std::vector<vector3_t> rootAngularVelocities_;

    // Current frame index for velocity lookup
    mutable int currentFrameIdx_;
};

}  // namespace g1
}  // namespace tbai
