#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <spdlog/logger.h>
#include <tbai_estim/inekf/Observations.hpp>
#include <tbai_estim/vision/PointCloudIndexer.hpp>

namespace tbai {
namespace vision {

class SuperPointExtractor;

// Frame-to-frame visual-feature tracker. Assigns persistent integer IDs to
// matched features across consecutive frames so that
// InEKFEstimator::correctVisualLandmarks can do measurement updates on the
// tracked landmarks (and only augments the state for genuinely new ones).
//
// The 3D position of each landmark is sampled from a PointCloudIndexer at
// the keypoint's pixel; keypoints with no valid depth are dropped before
// matching.
//
// Matching is L2-distance nearest-neighbour with Lowe's ratio test. Inputs
// are SuperPoint-style descriptors but any L2-normalized descriptor of the
// same dim works.
class VisualLandmarkTracker {
   public:
    struct Options {
        float ratioThreshold = 0.75f;          // Lowe ratio: 2nd-best / best > threshold -> reject
        float maxDescriptorDistance = 1.0f;    // L2 distance cap on best match (descriptors L2-normalized -> max 2)
        double landmarkVariance = 0.01;         // diagonal entry of the per-axis landmark covariance (m^2)
        int firstId = 1;                       // ids must be > 0 for ::inekf::Landmark
    };

    explicit VisualLandmarkTracker(const Options &options);
    VisualLandmarkTracker();

    // Process a single frame. Returns landmarks ready to feed to
    // InEKFEstimator::correctVisualLandmarks. `keypoints` and `descriptors`
    // (N x D, float32) must have matching row counts.
    ::inekf::vectorLandmarks processFrame(const std::vector<cv::Point2f> &keypoints, const cv::Mat &descriptors,
                                          const PointCloudIndexer &depth);

    // Convenience overload that runs SuperPoint internally.
    ::inekf::vectorLandmarks processFrame(SuperPointExtractor &extractor, const cv::Mat &image,
                                          const PointCloudIndexer &depth);

    // Forget all tracks (e.g., on filter reset).
    void reset();

    int trackedCount() const { return prevDescriptors_.rows; }
    int nextId() const { return nextId_; }

   private:
    Options options_;
    cv::Mat prevDescriptors_;            // M x D
    std::vector<int> prevIds_;            // size M
    int nextId_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace vision
}  // namespace tbai
