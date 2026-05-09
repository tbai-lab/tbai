// End-to-end smoke test for the visual-odometry pipeline:
//
//   image  ->  SuperPointExtractor  ->  keypoints + descriptors
//   cloud  ->  PointCloudIndexer    ->  3D lookup
//                       |
//                       v
//                VisualLandmarkTracker
//                       |
//                       v
//          ::inekf::vectorLandmarks
//                       |
//                       v
//                InEKF::CorrectLandmarks
//
// Skipped unless TBAI_SUPERPOINT_ONNX points to a real model file.

#include <cstdint>
#include <cstdlib>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tbai_estim/inekf/InEKF.hpp>
#include <tbai_estim/inekf/RobotState.hpp>
#include <tbai_estim/vision/PointCloudIndexer.hpp>
#include <tbai_estim/vision/SuperPointExtractor.hpp>
#include <tbai_estim/vision/VisualLandmarkTracker.hpp>

namespace {

constexpr int IMG_W = 640;
constexpr int IMG_H = 480;

cv::Mat makeTexturedImage(int seed) {
    cv::Mat img(IMG_H, IMG_W, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(img, cv::RNG::UNIFORM, 0, 256);
    // Add a few high-contrast shapes for guaranteed corners.
    for (int i = 0; i < 30; ++i) {
        int x = rng.uniform(20, IMG_W - 30);
        int y = rng.uniform(20, IMG_H - 30);
        int s = rng.uniform(8, 20);
        cv::rectangle(img, cv::Rect(x, y, s, s), cv::Scalar(rng.uniform(0, 64) ? 0 : 255), -1);
    }
    return img;
}

// Cloud aligned to the image: pixel (u, v) -> (X, Y, Z) where Z is constant
// depth and X/Y come from a pinhole back-projection. No metric realism needed
// — the test only cares that the tracker + InEKF see consistent 3D positions.
std::vector<std::uint8_t> makeAlignedCloud(int height, int width, double depth) {
    const double fx = 500.0;
    const double fy = 500.0;
    const double cx = width / 2.0;
    const double cy = height / 2.0;
    std::vector<std::uint8_t> buf(height * width * 3 * sizeof(float));
    auto *p = reinterpret_cast<float *>(buf.data());
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            int idx = (v * width + u) * 3;
            p[idx + 0] = static_cast<float>((u - cx) * depth / fx);
            p[idx + 1] = static_cast<float>((v - cy) * depth / fy);
            p[idx + 2] = static_cast<float>(depth);
        }
    }
    return buf;
}

const char *getModelPath() { return std::getenv("TBAI_SUPERPOINT_ONNX"); }

}  // namespace

TEST(VOEndToEnd, RepeatedFramePropagatesIdsAndCorrectionDoesNotMoveStaticPose) {
    const char *modelPath = getModelPath();
    if (!modelPath) {
        GTEST_SKIP() << "TBAI_SUPERPOINT_ONNX not set; skipping end-to-end test";
    }

    tbai::vision::SuperPointExtractor::Options spOpts;
    spOpts.inputWidth = IMG_W;
    spOpts.inputHeight = IMG_H;
    spOpts.maxKeypoints = 200;
    tbai::vision::SuperPointExtractor extractor(modelPath, spOpts);

    auto cloudBuf = makeAlignedCloud(IMG_H, IMG_W, /*depth=*/3.0);
    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), IMG_H, IMG_W, IMG_W * 3 * sizeof(float),
                                          3 * sizeof(float));

    cv::Mat image = makeTexturedImage(/*seed=*/1234);

    // Set up a low-uncertainty filter so any landmark-driven update is small
    // when the observation is consistent with the current pose.
    ::inekf::RobotState state;
    state.setRotation(Eigen::Matrix3d::Identity());
    state.setVelocity(Eigen::Vector3d::Zero());
    state.setPosition(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();
    P.diagonal().setConstant(1e-6);
    state.setP(P);
    ::inekf::InEKF filter(state);

    tbai::vision::VisualLandmarkTracker tracker;

    // Frame 1.
    auto landmarks1 = tracker.processFrame(extractor, image, cloud);
    ASSERT_GT(landmarks1.size(), 20u) << "SP should produce at least a few dozen keypoints with valid depth";
    Eigen::Vector3d posBefore = filter.getState().getPosition();
    filter.CorrectLandmarks(landmarks1);

    std::set<int> ids1;
    for (const auto &lm : landmarks1) ids1.insert(lm.id);

    // Frame 2: identical input -> tracker should re-match almost everything.
    auto landmarks2 = tracker.processFrame(extractor, image, cloud);
    ASSERT_GT(landmarks2.size(), 20u);

    int propagated = 0;
    for (const auto &lm : landmarks2) {
        if (ids1.count(lm.id)) ++propagated;
    }
    EXPECT_GT(propagated, static_cast<int>(landmarks2.size()) * 7 / 10)
        << "At least 70% of frame-1 ids should propagate to frame 2 on a static scene. "
        << "Got " << propagated << " / " << landmarks2.size();

    filter.CorrectLandmarks(landmarks2);
    Eigen::Vector3d posAfter = filter.getState().getPosition();

    // The filter started at truth and observations are consistent with truth, so
    // any drift should be tiny. Loose bound -- this is a smoke test.
    EXPECT_LT((posAfter - posBefore).norm(), 0.05);
}
