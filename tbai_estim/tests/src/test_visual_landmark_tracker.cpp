// Tests the descriptor-matching + ID-propagation logic of VisualLandmarkTracker
// in isolation, without needing a real SuperPoint ONNX model.
//
// We construct synthetic descriptors that look like normalized SuperPoint
// descriptors (random direction, unit norm), and a tiny organized point grid
// that maps each pixel to a known 3D position.

#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <tbai_estim/vision/PointCloudIndexer.hpp>
#include <tbai_estim/vision/VisualLandmarkTracker.hpp>

namespace {

constexpr int H = 32;
constexpr int W = 32;
constexpr int D = 32;  // descriptor dim

// Build a (H x W x 3) grid where pixel (u, v) maps to the 3D point (u, v, 5.0).
std::vector<std::uint8_t> makeFlatGridCloud() {
    std::vector<std::uint8_t> buf(H * W * 3 * sizeof(float));
    auto *p = reinterpret_cast<float *>(buf.data());
    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            int idx = (v * W + u) * 3;
            p[idx + 0] = static_cast<float>(u);
            p[idx + 1] = static_cast<float>(v);
            p[idx + 2] = 5.0f;
        }
    }
    return buf;
}

cv::Mat randomUnitDescriptor(std::mt19937 &rng, int dim) {
    std::normal_distribution<float> dist(0.0f, 1.0f);
    cv::Mat d(1, dim, CV_32FC1);
    float *p = d.ptr<float>(0);
    double sumSq = 0.0;
    for (int i = 0; i < dim; ++i) {
        p[i] = dist(rng);
        sumSq += p[i] * p[i];
    }
    float norm = static_cast<float>(std::sqrt(sumSq));
    for (int i = 0; i < dim; ++i) p[i] /= norm;
    return d;
}

}  // namespace

TEST(VisualLandmarkTracker, FirstFrameAssignsFreshIds) {
    auto cloudBuf = makeFlatGridCloud();
    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));

    std::mt19937 rng(0xC0FFEE);
    const int N = 5;
    std::vector<cv::Point2f> kps;
    cv::Mat desc(N, D, CV_32FC1);
    for (int i = 0; i < N; ++i) {
        kps.emplace_back(2.0f + i * 5.0f, 3.0f + i * 4.0f);
        randomUnitDescriptor(rng, D).copyTo(desc.row(i));
    }

    tbai::vision::VisualLandmarkTracker tracker;
    auto landmarks = tracker.processFrame(kps, desc, cloud);
    ASSERT_EQ(landmarks.size(), N);
    std::set<int> seen;
    for (const auto &lm : landmarks) {
        EXPECT_GT(lm.id, 0);
        EXPECT_TRUE(seen.insert(lm.id).second) << "id " << lm.id << " repeated";
        // Position should match the synthetic grid: (u, v, 5).
        EXPECT_NEAR(lm.position[2], 5.0, 1e-6);
    }
    EXPECT_EQ(tracker.trackedCount(), N);
}

TEST(VisualLandmarkTracker, IdenticalSecondFramePropagatesAllIds) {
    auto cloudBuf = makeFlatGridCloud();
    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));

    std::mt19937 rng(0xF00D);
    const int N = 8;
    std::vector<cv::Point2f> kps;
    cv::Mat desc(N, D, CV_32FC1);
    for (int i = 0; i < N; ++i) {
        kps.emplace_back(3.0f + i * 3.0f, 4.0f + i * 2.0f);
        randomUnitDescriptor(rng, D).copyTo(desc.row(i));
    }

    tbai::vision::VisualLandmarkTracker tracker;
    auto first = tracker.processFrame(kps, desc, cloud);
    std::vector<int> firstIds;
    for (const auto &lm : first) firstIds.push_back(lm.id);

    auto second = tracker.processFrame(kps, desc.clone(), cloud);
    ASSERT_EQ(second.size(), first.size());
    for (size_t i = 0; i < second.size(); ++i) {
        EXPECT_EQ(second[i].id, firstIds[i]) << "id at slot " << i << " was not propagated";
    }
}

TEST(VisualLandmarkTracker, NewDescriptorsGetFreshIds) {
    auto cloudBuf = makeFlatGridCloud();
    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));

    std::mt19937 rng(0xDEAD);
    const int N = 4;
    std::vector<cv::Point2f> kps;
    cv::Mat desc(N, D, CV_32FC1);
    for (int i = 0; i < N; ++i) {
        kps.emplace_back(2.0f + i * 6.0f, 5.0f + i * 3.0f);
        randomUnitDescriptor(rng, D).copyTo(desc.row(i));
    }

    tbai::vision::VisualLandmarkTracker tracker;
    auto first = tracker.processFrame(kps, desc, cloud);
    std::set<int> firstIds;
    for (const auto &lm : first) firstIds.insert(lm.id);

    // Replace all descriptors with brand-new random ones at the same pixels.
    cv::Mat desc2(N, D, CV_32FC1);
    for (int i = 0; i < N; ++i) {
        randomUnitDescriptor(rng, D).copyTo(desc2.row(i));
    }
    auto second = tracker.processFrame(kps, desc2, cloud);
    for (const auto &lm : second) {
        EXPECT_EQ(firstIds.count(lm.id), 0u) << "id " << lm.id << " should be fresh, not reused";
    }
}

TEST(VisualLandmarkTracker, DropsKeypointsWithoutDepth) {
    // Mark a couple of pixels invalid by writing NaN into the cloud.
    auto cloudBuf = makeFlatGridCloud();
    auto *p = reinterpret_cast<float *>(cloudBuf.data());
    auto invalidate = [&](int u, int v) {
        int idx = (v * W + u) * 3;
        p[idx + 0] = std::numeric_limits<float>::quiet_NaN();
    };
    invalidate(10, 10);
    invalidate(20, 20);

    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));

    std::mt19937 rng(0xBEEF);
    std::vector<cv::Point2f> kps = {
        {10.0f, 10.0f},  // dropped
        {15.0f, 15.0f},  // kept
        {20.0f, 20.0f},  // dropped
        {25.0f, 25.0f},  // kept
    };
    cv::Mat desc(static_cast<int>(kps.size()), D, CV_32FC1);
    for (int i = 0; i < static_cast<int>(kps.size()); ++i) {
        randomUnitDescriptor(rng, D).copyTo(desc.row(i));
    }

    tbai::vision::VisualLandmarkTracker tracker;
    auto out = tracker.processFrame(kps, desc, cloud);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_NEAR(out[0].position[0], 15.0, 1e-6);
    EXPECT_NEAR(out[1].position[0], 25.0, 1e-6);
}

TEST(VisualLandmarkTracker, ResetForgetsAllTracks) {
    auto cloudBuf = makeFlatGridCloud();
    tbai::vision::PointCloudIndexer cloud(cloudBuf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));

    std::mt19937 rng(0x42);
    const int N = 3;
    std::vector<cv::Point2f> kps;
    cv::Mat desc(N, D, CV_32FC1);
    for (int i = 0; i < N; ++i) {
        kps.emplace_back(5.0f + i * 4.0f, 5.0f + i * 4.0f);
        randomUnitDescriptor(rng, D).copyTo(desc.row(i));
    }

    tbai::vision::VisualLandmarkTracker tracker;
    auto first = tracker.processFrame(kps, desc, cloud);
    EXPECT_EQ(tracker.trackedCount(), N);

    tracker.reset();
    EXPECT_EQ(tracker.trackedCount(), 0);

    auto afterReset = tracker.processFrame(kps, desc, cloud);
    // ids should restart from firstId; no propagation possible.
    for (size_t i = 0; i < afterReset.size(); ++i) {
        EXPECT_EQ(afterReset[i].id, first[i].id) << "after reset, ids restart from the same base";
    }
}
