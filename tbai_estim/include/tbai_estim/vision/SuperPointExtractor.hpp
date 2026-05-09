#pragma once

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <spdlog/logger.h>

namespace tbai {
namespace vision {

// Thin C++ wrapper around an ONNX-exported SuperPoint model.
//
// Expected ONNX I/O (configurable via Options):
//   input  "image"        float32  [1, 1, H, W]   grayscale, normalized to [0, 1]
//   output "keypoints"    float32  [1, N, 2]      sub-pixel (x, y), in network input coords
//   output "scores"       float32  [1, N]
//   output "descriptors"  float32  [1, N, D]      D defaults to 256
//
// The exporter is responsible for emitting a fixed N (typically via top-k);
// the extractor here sub-selects further by scoreThreshold and maxKeypoints.
//
// Run the helper at tbai_estim/scripts/export_superpoint_onnx.py to produce a
// compatible model from kornia's pretrained SuperPoint weights.
class SuperPointExtractor {
   public:
    struct Options {
        int inputWidth = 640;
        int inputHeight = 480;
        float scoreThreshold = 0.005f;
        int maxKeypoints = 1024;
        int descriptorDim = 256;
        std::string inputName = "image";
        std::string keypointsOutputName = "keypoints";
        std::string scoresOutputName = "scores";
        std::string descriptorsOutputName = "descriptors";
        int intraOpThreads = 1;
    };

    struct Output {
        std::vector<cv::Point2f> keypoints;  // in original-image coordinates
        std::vector<float> scores;
        cv::Mat descriptors;  // N x D, float32, L2-normalized rows
    };

    explicit SuperPointExtractor(const std::string &onnxPath);
    SuperPointExtractor(const std::string &onnxPath, const Options &options);
    ~SuperPointExtractor();

    SuperPointExtractor(const SuperPointExtractor &) = delete;
    SuperPointExtractor &operator=(const SuperPointExtractor &) = delete;

    // Detect keypoints + descriptors on an image. Accepts CV_8UC1 (grayscale)
    // or CV_8UC3 (BGR). Keypoint coordinates are returned in the input image's
    // pixel frame (i.e. rescaled out of the network's resize).
    Output detect(const cv::Mat &image);

    int inputWidth() const { return options_.inputWidth; }
    int inputHeight() const { return options_.inputHeight; }
    int descriptorDim() const { return options_.descriptorDim; }

   private:
    Options options_;
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
    std::vector<const char *> inputNamesC_;
    std::vector<const char *> outputNamesC_;
    int outputN_ = -1;  // determined on first inference from output shape

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace vision
}  // namespace tbai
