#pragma once

#include <memory>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_estim/inekf/InEKF.hpp>

namespace tbai {
namespace inekf {
class InEKFEstimator {
   public:
    InEKFEstimator(std::vector<std::string> footNames, const std::string &urdf = "");

    void update(scalar_t currentTime, scalar_t dt, const vector4_t &quatBase, const vector_t &jointPositions,
                const vector_t &jointVelocities, const vector3_t &linearAccBase, const vector3_t &angularVelBase,
                std::vector<bool> contacts, bool rectifyOrientation = true, bool enablePositionEstimation = true);

    // Apply a vision-derived landmark correction to the filter. Each landmark
    // carries an integer ID; landmarks with IDs already tracked by the filter
    // produce a measurement update, while new IDs are added to the state. The
    // landmark `position` is the 3D position of the landmark in the robot
    // (body) frame, with `covariance` expressed in the same frame. See
    // ::inekf::InEKF::CorrectLandmarks for the full update semantics.
    //
    // When `pruneStale` is true, any tracked landmark NOT in the current
    // observation set is removed from the filter state. This keeps the state
    // matrix bounded and is essential for real-time use -- without it, every
    // unmatched SuperPoint keypoint adds 3 rows/cols to the covariance and the
    // O(N^2) update cost grows unboundedly across frames.
    void correctVisualLandmarks(const ::inekf::vectorLandmarks &landmarks, bool pruneStale = true);

    void computeLinPosVel(scalar_t currentTime, scalar_t dt, Eigen::Vector3d &acc, Eigen::Matrix3d &w_R_b,
                          Eigen::Vector3d &v_b);

    void setupPinocchioModel(const std::string &urdf = "");

    inline vector3_t getBasePosition() { return inekf_.getState().getPosition(); }
    inline vector3_t getBaseVelocity() { return inekf_.getState().getVelocity(); }
    inline vector4_t getBaseOrientation() { return quaternion_t(inekf_.getState().getRotation()).coeffs(); }
    inline vector3_t getGyroscopeBias() { return inekf_.getState().getGyroscopeBias(); }
    inline ::inekf::RobotState &getState() { return inekf_.getState(); }

    pinocchio::Model model_;
    pinocchio::Data data_;

    ::inekf::InEKF inekf_;
    std::vector<size_t> legIndices_;

    bool firstMessageReceived_ = false;
    vector3_t lastAngularVelocity_;
    vector3_t lastLinearAcceleration_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace inekf
}  // namespace tbai