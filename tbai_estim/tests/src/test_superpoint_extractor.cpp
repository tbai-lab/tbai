// Smoke test for SuperPointExtractor.
//
// This test runs the real extractor against an ONNX file, but the file is
// expensive to download/produce, so the test SKIPS when the env var
// `TBAI_SUPERPOINT_ONNX` is unset. Run as:
//
//   TBAI_SUPERPOINT_ONNX=/path/to/superpoint.onnx ctest -R test_superpoint_extractor --output-on-failure
//
// Produce the ONNX with tbai_estim/scripts/export_superpoint_onnx.py.

#include <cstdlib>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tbai_estim/vision/SuperPointExtractor.hpp>

namespace {

cv::Mat makeSyntheticImage(int width, int height) {
    // Random texture: SuperPoint should find SOME corner-like keypoints on it.
    cv::Mat img(height, width, CV_8UC1);
    cv::randu(img, 0, 255);
    // Sprinkle a few high-contrast squares for guaranteed corners.
    cv::rectangle(img, cv::Rect(50, 50, 30, 30), cv::Scalar(255), -1);
    cv::rectangle(img, cv::Rect(200, 100, 30, 30), cv::Scalar(0), -1);
    cv::rectangle(img, cv::Rect(400, 300, 30, 30), cv::Scalar(255), -1);
    return img;
}

const char *getModelPath() {
    return std::getenv("TBAI_SUPERPOINT_ONNX");
}

}  // namespace

TEST(SuperPointExtractor, LoadsAndDetectsKeypoints) {
    const char *modelPath = getModelPath();
    if (!modelPath) {
        GTEST_SKIP() << "TBAI_SUPERPOINT_ONNX not set; skipping (run scripts/export_superpoint_onnx.py first)";
    }

    tbai::vision::SuperPointExtractor::Options opts;
    opts.inputWidth = 640;
    opts.inputHeight = 480;
    opts.scoreThreshold = 0.005f;
    opts.maxKeypoints = 256;
    tbai::vision::SuperPointExtractor sp(modelPath, opts);

    cv::Mat img = makeSyntheticImage(640, 480);
    auto out = sp.detect(img);
    EXPECT_GT(out.keypoints.size(), 0u);
    EXPECT_EQ(out.keypoints.size(), out.scores.size());
    EXPECT_EQ(static_cast<size_t>(out.descriptors.rows), out.keypoints.size());
    EXPECT_EQ(out.descriptors.type(), CV_32FC1);

    // Keypoints should land inside the image.
    for (const auto &kp : out.keypoints) {
        EXPECT_GE(kp.x, 0.0f);
        EXPECT_LT(kp.x, static_cast<float>(img.cols));
        EXPECT_GE(kp.y, 0.0f);
        EXPECT_LT(kp.y, static_cast<float>(img.rows));
    }
}

TEST(SuperPointExtractor, RejectsBadImage) {
    const char *modelPath = getModelPath();
    if (!modelPath) {
        GTEST_SKIP() << "TBAI_SUPERPOINT_ONNX not set";
    }
    tbai::vision::SuperPointExtractor sp(modelPath);
    EXPECT_THROW(sp.detect(cv::Mat()), std::invalid_argument);
    cv::Mat wrongType(100, 100, CV_32FC3);
    EXPECT_THROW(sp.detect(wrongType), std::invalid_argument);
}
