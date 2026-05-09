// Synthetic test for the visual-landmark correction path.
//
// We exercise ::inekf::InEKF::CorrectLandmarks directly (rather than via
// InEKFEstimator) because the latter would require a Pinocchio URDF for foot
// kinematics, which is overkill for verifying the landmark-update behaviour.
// InEKFEstimator::correctVisualLandmarks is a one-line forward to
// CorrectLandmarks; an end-to-end test that exercises the full wrapper lives
// in the higher-level integration test that consumes images + point clouds.
//
// We model the visual-odometry workflow:
//   1. observe a set of landmarks for the first time -> they are augmented
//      into the filter state at (current_estimate + body_measurement),
//   2. drift the filter's position estimate (simulating IMU drift between
//      keyframes),
//   3. observe the same landmarks again with body-frame measurements that are
//      still consistent with the *true* pose,
//   4. expect the second correction to pull the position estimate back toward
//      truth.

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <tbai_estim/inekf/InEKF.hpp>
#include <tbai_estim/inekf/Observations.hpp>
#include <tbai_estim/inekf/RobotState.hpp>

namespace {

::inekf::RobotState makeState(const Eigen::Vector3d &position, double posVariance) {
    ::inekf::RobotState state;
    state.setRotation(Eigen::Matrix3d::Identity());
    state.setVelocity(Eigen::Vector3d::Zero());
    state.setPosition(position);

    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();
    P.block<3, 3>(0, 0).diagonal().setConstant(1e-4);             // rotation
    P.block<3, 3>(3, 3).diagonal().setConstant(1e-4);             // velocity
    P.block<3, 3>(6, 6).diagonal().setConstant(posVariance);       // position
    P.block<3, 3>(9, 9).diagonal().setConstant(1e-6);             // gyro bias
    P.block<3, 3>(12, 12).diagonal().setConstant(1e-6);           // accel bias
    state.setP(P);
    return state;
}

}  // namespace

TEST(VisualLandmarkCorrection, NoLandmarksLeavesStateUnchanged) {
    ::inekf::InEKF filter(makeState(Eigen::Vector3d(1.0, 2.0, 0.5), 0.25));
    Eigen::Vector3d before = filter.getState().getPosition();
    filter.CorrectLandmarks({});
    Eigen::Vector3d after = filter.getState().getPosition();
    EXPECT_TRUE(before.isApprox(after));
}

TEST(VisualLandmarkCorrection, RepeatedIdAcrossFramesUpdatesEstimatedLandmark) {
    ::inekf::InEKF filter(makeState(Eigen::Vector3d::Zero(), 0.1));

    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-3;
    ::inekf::vectorLandmarks first;
    first.emplace_back(42, Eigen::Vector3d(2.0, 0.0, 0.0), cov);
    filter.CorrectLandmarks(first);

    auto estimatedAfterFirst = filter.getEstimatedLandmarks();
    ASSERT_EQ(estimatedAfterFirst.count(42), 1u);
    int stateIdx = estimatedAfterFirst.at(42);

    ::inekf::vectorLandmarks second;
    second.emplace_back(42, Eigen::Vector3d(2.0, 0.0, 0.0), cov);
    filter.CorrectLandmarks(second);

    auto estimatedAfterSecond = filter.getEstimatedLandmarks();
    EXPECT_EQ(estimatedAfterSecond.count(42), 1u);
    EXPECT_EQ(estimatedAfterSecond.at(42), stateIdx) << "State index for tracked landmark must not change";
}

TEST(VisualLandmarkCorrection, ReobservationCorrectsDriftedPosition) {
    // Step 1: observe landmarks for the first time at the true pose with low
    // position uncertainty -> the augmented landmarks inherit a tight covariance
    // and a tight cross-covariance with the robot position.
    Eigen::Vector3d truePos(1.0, -0.5, 0.3);
    ::inekf::InEKF filter(makeState(truePos, 1e-6));

    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-4;
    std::vector<std::pair<int, Eigen::Vector3d>> bodyMeas = {
        {1, Eigen::Vector3d(2.0, 0.0, 0.0)},
        {2, Eigen::Vector3d(0.0, 2.5, 0.1)},
        {3, Eigen::Vector3d(-1.5, 1.0, -0.2)},
        {4, Eigen::Vector3d(1.0, -2.0, 0.4)},
        {5, Eigen::Vector3d(3.0, 0.5, 0.0)},
        {6, Eigen::Vector3d(0.5, -3.0, 0.1)},
    };
    {
        ::inekf::vectorLandmarks meas;
        for (const auto &kv : bodyMeas) meas.emplace_back(kv.first, kv.second, cov);
        filter.CorrectLandmarks(meas);
    }

    ASSERT_EQ(filter.getEstimatedLandmarks().size(), bodyMeas.size());

    // Step 2: simulate IMU drift between observations. We add UNCORRELATED
    // position uncertainty (mirroring what InEKF::Propagate would do over time)
    // and shift the estimate. Without this uncorrelated term the residual is
    // perfectly explained by a common-mode shift of robot+landmark and is
    // unobservable -- which is correct EKF behaviour.
    Eigen::Vector3d drift(0.20, -0.10, 0.05);
    {
        auto state = filter.getState();
        state.setPosition(truePos + drift);
        Eigen::MatrixXd P = state.getP();
        P.block<3, 3>(6, 6) += Eigen::Matrix3d::Identity() * 0.5;  // uncorrelated growth
        state.setP(P);
        filter.setState(state);
    }

    double errBefore = (filter.getState().getPosition() - truePos).norm();
    ASSERT_NEAR(errBefore, drift.norm(), 1e-9);

    // Step 3: re-observe the same landmarks with body-frame measurements still
    // consistent with the true pose.
    {
        ::inekf::vectorLandmarks meas;
        for (const auto &kv : bodyMeas) meas.emplace_back(kv.first, kv.second, cov);
        filter.CorrectLandmarks(meas);
    }

    double errAfter = (filter.getState().getPosition() - truePos).norm();
    EXPECT_LT(errAfter, errBefore) << "Re-observation should reduce position error. "
                                   << "Before: " << errBefore << "  After: " << errAfter;
    // With many consistent observations and tight initial landmark covariance,
    // drift should be largely corrected.
    EXPECT_LT(errAfter, 0.5 * errBefore);
}
