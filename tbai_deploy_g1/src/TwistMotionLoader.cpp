#include "tbai_deploy_g1/TwistMotionLoader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tbai {
namespace g1 {

TwistMotionLoader::TwistMotionLoader(const std::string &motionFile, float fps, const std::vector<int> &twistJointIds)
    : MotionLoader(motionFile, fps), twistJointIds_(twistJointIds), currentFrameIdx_(0) {
    if (static_cast<int>(twistJointIds_.size()) != TWIST_ACTION_MIMIC_DOF) {
        throw std::runtime_error("TwistMotionLoader: expected " + std::to_string(TWIST_ACTION_MIMIC_DOF) +
                                 " twist joint IDs, got " + std::to_string(twistJointIds_.size()));
    }

    // Compute root velocities from position derivatives
    computeVelocities();
}

void TwistMotionLoader::computeVelocities() {
    const int numFrames = MotionLoader::numFrames();
    const float dt = MotionLoader::dt();

    rootLinearVelocities_.resize(numFrames);
    rootAngularVelocities_.resize(numFrames);

    // Compute linear velocities from position derivatives
    for (int i = 0; i < numFrames - 1; ++i) {
        MotionLoader::update(i * dt);
        vector3_t pos0 = MotionLoader::rootPosition();
        MotionLoader::update((i + 1) * dt);
        vector3_t pos1 = MotionLoader::rootPosition();

        // Linear velocity in world frame
        rootLinearVelocities_[i] = (pos1 - pos0) / dt;
    }
    // Last frame uses same velocity as second-to-last
    if (numFrames > 1) {
        rootLinearVelocities_[numFrames - 1] = rootLinearVelocities_[numFrames - 2];
    } else {
        rootLinearVelocities_[0].setZero();
    }

    // Compute angular velocities from quaternion derivatives
    for (int i = 0; i < numFrames - 1; ++i) {
        MotionLoader::update(i * dt);
        quaternion_t q0 = MotionLoader::rootQuaternion();
        MotionLoader::update((i + 1) * dt);
        quaternion_t q1 = MotionLoader::rootQuaternion();

        // Angular velocity from quaternion derivative: omega = 2 * q_dot * q^(-1)
        quaternion_t dq = quaternion_t(q1.w() - q0.w(), q1.x() - q0.x(), q1.y() - q0.y(), q1.z() - q0.z());

        // omega = 2 * dq/dt * q^-1 (imaginary part)
        quaternion_t omega_q = quaternion_t(dq.w() / dt, dq.x() / dt, dq.y() / dt, dq.z() / dt) * q0.conjugate();
        rootAngularVelocities_[i] = vector3_t(2.0 * omega_q.x(), 2.0 * omega_q.y(), 2.0 * omega_q.z());
    }
    // Last frame uses same velocity as second-to-last
    if (numFrames > 1) {
        rootAngularVelocities_[numFrames - 1] = rootAngularVelocities_[numFrames - 2];
    } else {
        rootAngularVelocities_[0].setZero();
    }

    // Reset to initial state
    MotionLoader::update(0.0f);
}

vector_t TwistMotionLoader::getActionMimic() const {
    // action_mimic: [height(1), RPY(3), lin_vel(3), yaw_ang_vel(1), dof_pos(23)] = 31 dims
    vector_t actionMimic(TWIST_ACTION_MIMIC_SIZE);

    // Height (z-position)
    actionMimic[0] = rootHeight();

    // RPY
    vector3_t rpy = rootRPY();
    actionMimic.segment<3>(1) = rpy;

    // Linear velocity in local frame
    vector3_t linVelLocal = rootLinearVelocity();
    actionMimic.segment<3>(4) = linVelLocal;

    // Yaw angular velocity
    vector3_t angVel = rootAngularVelocity();
    actionMimic[7] = angVel[2];  // yaw rate

    // DOF positions (23 TWIST joints)
    actionMimic.segment(8, TWIST_ACTION_MIMIC_DOF) = jointPosTwist();

    return actionMimic;
}

vector_t TwistMotionLoader::jointPosTwist() const {
    // Get full 29-DOF joint positions from motion
    vector_t fullJointPos = MotionLoader::jointPos();

    // Extract 23 TWIST joints using the mapping
    vector_t twistJointPos(TWIST_ACTION_MIMIC_DOF);
    for (int i = 0; i < TWIST_ACTION_MIMIC_DOF; ++i) {
        twistJointPos[i] = fullJointPos[twistJointIds_[i]];
    }

    return twistJointPos;
}

vector_t TwistMotionLoader::jointVelTwist() const {
    // Get full 29-DOF joint velocities from motion
    vector_t fullJointVel = MotionLoader::jointVel();

    // Extract 23 TWIST joints using the mapping
    vector_t twistJointVel(TWIST_ACTION_MIMIC_DOF);
    for (int i = 0; i < TWIST_ACTION_MIMIC_DOF; ++i) {
        twistJointVel[i] = fullJointVel[twistJointIds_[i]];
    }

    return twistJointVel;
}

vector3_t TwistMotionLoader::rootLinearVelocity() const {
    // Get current root quaternion to transform world velocity to local
    quaternion_t rootQuat = MotionLoader::rootQuaternion();
    matrix3_t worldToLocal = rootQuat.conjugate().toRotationMatrix();

    // Use current frame index for velocity lookup
    if (!rootLinearVelocities_.empty()) {
        int idx = std::clamp(currentFrameIdx_, 0, static_cast<int>(rootLinearVelocities_.size()) - 1);
        return worldToLocal * rootLinearVelocities_[idx];
    }
    return vector3_t::Zero();
}

vector3_t TwistMotionLoader::rootAngularVelocity() const {
    if (!rootAngularVelocities_.empty()) {
        int idx = std::clamp(currentFrameIdx_, 0, static_cast<int>(rootAngularVelocities_.size()) - 1);
        return rootAngularVelocities_[idx];
    }
    return vector3_t::Zero();
}

scalar_t TwistMotionLoader::rootHeight() const {
    return MotionLoader::rootPosition()[2];
}

vector3_t TwistMotionLoader::rootRPY() const {
    quaternion_t quat = MotionLoader::rootQuaternion();

    // Convert quaternion to RPY (ZYX convention)
    // Roll (x-axis rotation)
    scalar_t sinr_cosp = 2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
    scalar_t cosr_cosp = 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
    scalar_t roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    scalar_t sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
    scalar_t pitch;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);  // use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    scalar_t siny_cosp = 2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
    scalar_t cosy_cosp = 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
    scalar_t yaw = std::atan2(siny_cosp, cosy_cosp);

    return vector3_t(roll, pitch, yaw);
}

}  // namespace g1
}  // namespace tbai
