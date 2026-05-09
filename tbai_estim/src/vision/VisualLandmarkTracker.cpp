#include <tbai_estim/vision/VisualLandmarkTracker.hpp>

#include <stdexcept>
#include <utility>

#include <opencv2/features2d.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_estim/vision/SuperPointExtractor.hpp>

namespace tbai {
namespace vision {

VisualLandmarkTracker::VisualLandmarkTracker(const Options &options) : options_(options), nextId_(options.firstId) {
    logger_ = tbai::getLogger("visual_landmark_tracker");
    if (options_.firstId <= 0) {
        // Landmark ids of 0 collide with the InEKF's "no contact" sentinel; bump.
        options_.firstId = 1;
        nextId_ = 1;
    }
}

VisualLandmarkTracker::VisualLandmarkTracker() : VisualLandmarkTracker(Options{}) {}

void VisualLandmarkTracker::reset() {
    prevDescriptors_.release();
    prevIds_.clear();
    nextId_ = options_.firstId;
}

::inekf::vectorLandmarks VisualLandmarkTracker::processFrame(const std::vector<cv::Point2f> &keypoints,
                                                             const cv::Mat &descriptors,
                                                             const PointCloudIndexer &depth) {
    if (static_cast<int>(keypoints.size()) != descriptors.rows) {
        throw std::invalid_argument("VisualLandmarkTracker: keypoints and descriptors row count mismatch");
    }
    if (descriptors.type() != CV_32FC1) {
        throw std::invalid_argument("VisualLandmarkTracker: descriptors must be CV_32FC1");
    }

    // Step 1: drop keypoints without valid depth, build a "live" subset.
    std::vector<int> liveSrcIdx;
    std::vector<Eigen::Vector3d> livePos;
    liveSrcIdx.reserve(keypoints.size());
    livePos.reserve(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        auto p = depth.at(keypoints[i].x, keypoints[i].y);
        if (!p) continue;
        liveSrcIdx.push_back(static_cast<int>(i));
        livePos.push_back(*p);
    }

    cv::Mat liveDesc(static_cast<int>(liveSrcIdx.size()), descriptors.cols, CV_32FC1);
    for (int k = 0; k < static_cast<int>(liveSrcIdx.size()); ++k) {
        descriptors.row(liveSrcIdx[k]).copyTo(liveDesc.row(k));
    }

    // Step 2: match against previous frame's descriptors.
    std::vector<int> ids(liveSrcIdx.size(), -1);
    if (!prevDescriptors_.empty() && !liveDesc.empty()) {
        cv::BFMatcher matcher(cv::NORM_L2, /*crossCheck=*/false);
        std::vector<std::vector<cv::DMatch>> knn;
        // Query = live (current) descriptors, Train = prev. queryIdx is current, trainIdx is prev.
        matcher.knnMatch(liveDesc, prevDescriptors_, knn, /*k=*/2);

        // Keep best match per previous-id so two current keypoints can't claim the same prev id.
        std::vector<int> prevToCurr(prevDescriptors_.rows, -1);
        std::vector<float> prevToCurrDist(prevDescriptors_.rows, std::numeric_limits<float>::max());
        for (const auto &m : knn) {
            if (m.empty()) continue;
            const auto &best = m[0];
            if (best.distance > options_.maxDescriptorDistance) continue;
            // Lowe ratio (only meaningful when we have a 2nd nearest).
            if (m.size() >= 2 && best.distance > options_.ratioThreshold * m[1].distance) continue;

            int currIdx = best.queryIdx;
            int prevIdx = best.trainIdx;
            if (best.distance < prevToCurrDist[prevIdx]) {
                // Reclaim previous winner if any.
                int oldCurr = prevToCurr[prevIdx];
                if (oldCurr >= 0) ids[oldCurr] = -1;
                prevToCurr[prevIdx] = currIdx;
                prevToCurrDist[prevIdx] = best.distance;
                ids[currIdx] = prevIds_[prevIdx];
            }
        }
    }

    // Step 3: assign fresh ids to unmatched live keypoints.
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 0) {
            ids[i] = nextId_++;
        }
    }

    // Step 4: build landmark observations.
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * options_.landmarkVariance;
    ::inekf::vectorLandmarks out;
    out.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        out.emplace_back(ids[i], livePos[i], cov);
    }

    // Step 5: rotate state forward.
    prevDescriptors_ = liveDesc.clone();
    prevIds_ = std::move(ids);

    return out;
}

::inekf::vectorLandmarks VisualLandmarkTracker::processFrame(SuperPointExtractor &extractor, const cv::Mat &image,
                                                             const PointCloudIndexer &depth) {
    auto out = extractor.detect(image);
    return processFrame(out.keypoints, out.descriptors, depth);
}

}  // namespace vision
}  // namespace tbai
