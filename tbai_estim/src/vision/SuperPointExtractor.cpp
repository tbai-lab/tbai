#include <tbai_estim/vision/SuperPointExtractor.hpp>

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <tbai_core/Logging.hpp>

namespace tbai {
namespace vision {

namespace {

void copyName(const Ort::AllocatedStringPtr &src, std::vector<const char *> &dst) {
    dst.push_back(strdup(src.get()));
}

}  // namespace

SuperPointExtractor::SuperPointExtractor(const std::string &onnxPath) : SuperPointExtractor(onnxPath, Options{}) {}

SuperPointExtractor::SuperPointExtractor(const std::string &onnxPath, const Options &options) : options_(options) {
    logger_ = tbai::getLogger("superpoint_extractor");
    TBAI_LOG_INFO(logger_, "Loading SuperPoint ONNX model from {}", onnxPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "SuperPointExtractor");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(options_.intraOpThreads);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, onnxPath.c_str(), sessionOptions);
    memoryInfo_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions allocator;

    // Sanity-check that the user-supplied input/output names exist on the model.
    auto containsName = [&](auto countFn, auto nameFn, const std::string &needle) {
        size_t n = (ortSession_.get()->*countFn)();
        for (size_t i = 0; i < n; ++i) {
            auto name = (ortSession_.get()->*nameFn)(i, allocator);
            if (needle == name.get()) return true;
        }
        return false;
    };

    if (!containsName(&Ort::Session::GetInputCount, &Ort::Session::GetInputNameAllocated, options_.inputName)) {
        throw std::runtime_error("SuperPointExtractor: ONNX model missing input '" + options_.inputName + "'");
    }
    for (const auto &outName :
         {options_.keypointsOutputName, options_.scoresOutputName, options_.descriptorsOutputName}) {
        if (!containsName(&Ort::Session::GetOutputCount, &Ort::Session::GetOutputNameAllocated, outName)) {
            throw std::runtime_error("SuperPointExtractor: ONNX model missing output '" + outName + "'");
        }
    }

    inputNamesC_.push_back(strdup(options_.inputName.c_str()));
    outputNamesC_.push_back(strdup(options_.keypointsOutputName.c_str()));
    outputNamesC_.push_back(strdup(options_.scoresOutputName.c_str()));
    outputNamesC_.push_back(strdup(options_.descriptorsOutputName.c_str()));

    TBAI_LOG_INFO(logger_, "SuperPoint loaded. Input size {}x{}, descriptor dim {}", options_.inputWidth,
                  options_.inputHeight, options_.descriptorDim);
}

SuperPointExtractor::~SuperPointExtractor() {
    for (auto *p : inputNamesC_) std::free(const_cast<char *>(p));
    for (auto *p : outputNamesC_) std::free(const_cast<char *>(p));
}

SuperPointExtractor::Output SuperPointExtractor::detect(const cv::Mat &image) {
    if (image.empty()) {
        throw std::invalid_argument("SuperPointExtractor::detect: empty image");
    }
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        throw std::invalid_argument("SuperPointExtractor::detect: image must be CV_8UC1 or CV_8UC3");
    }

    // Preprocess: BGR->gray, resize, normalize to [0, 1].
    cv::Mat gray;
    if (image.type() == CV_8UC3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(options_.inputWidth, options_.inputHeight), 0.0, 0.0, cv::INTER_AREA);
    cv::Mat normalized;
    resized.convertTo(normalized, CV_32FC1, 1.0 / 255.0);

    const int H = options_.inputHeight;
    const int W = options_.inputWidth;
    std::vector<int64_t> inputShape = {1, 1, H, W};
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(*memoryInfo_, normalized.ptr<float>(), H * W,
                                                             inputShape.data(), inputShape.size());

    auto outputs = ortSession_->Run(Ort::RunOptions{nullptr}, inputNamesC_.data(), &inputTensor, 1, outputNamesC_.data(),
                                    outputNamesC_.size());
    if (outputs.size() != 3) {
        throw std::runtime_error("SuperPointExtractor: expected 3 outputs, got " + std::to_string(outputs.size()));
    }

    auto &kpsT = outputs[0];
    auto &scoresT = outputs[1];
    auto &descT = outputs[2];

    auto kpsShape = kpsT.GetTensorTypeAndShapeInfo().GetShape();        // [1, N, 2]
    auto scoresShape = scoresT.GetTensorTypeAndShapeInfo().GetShape();  // [1, N]
    auto descShape = descT.GetTensorTypeAndShapeInfo().GetShape();      // [1, N, D]
    if (kpsShape.size() != 3 || kpsShape[2] != 2 || scoresShape.size() != 2 || descShape.size() != 3) {
        throw std::runtime_error("SuperPointExtractor: unexpected output ranks");
    }
    const int N = static_cast<int>(kpsShape[1]);
    const int D = static_cast<int>(descShape[2]);
    if (D != options_.descriptorDim) {
        TBAI_LOG_WARN(logger_, "SuperPoint descriptor dim mismatch: model={}, options={}. Using model dim.", D,
                      options_.descriptorDim);
        options_.descriptorDim = D;
    }
    outputN_ = N;

    // The reference SuperPoint export emits integer pixel keypoints; some
    // re-exports use float. Support both to stay robust.
    const auto kpsType = kpsT.GetTensorTypeAndShapeInfo().GetElementType();
    auto readKpx = [&](int i) -> float {
        if (kpsType == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
            return static_cast<float>(kpsT.GetTensorData<int64_t>()[i * 2 + 0]);
        }
        return kpsT.GetTensorData<float>()[i * 2 + 0];
    };
    auto readKpy = [&](int i) -> float {
        if (kpsType == ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
            return static_cast<float>(kpsT.GetTensorData<int64_t>()[i * 2 + 1]);
        }
        return kpsT.GetTensorData<float>()[i * 2 + 1];
    };
    const float *scoresData = scoresT.GetTensorData<float>();
    const float *descData = descT.GetTensorData<float>();

    // Filter by score threshold and (optionally) cap at maxKeypoints by sorting desc.
    std::vector<int> indices;
    indices.reserve(N);
    for (int i = 0; i < N; ++i) {
        if (scoresData[i] >= options_.scoreThreshold) {
            indices.push_back(i);
        }
    }
    if (static_cast<int>(indices.size()) > options_.maxKeypoints && options_.maxKeypoints > 0) {
        std::partial_sort(indices.begin(), indices.begin() + options_.maxKeypoints, indices.end(),
                          [&](int a, int b) { return scoresData[a] > scoresData[b]; });
        indices.resize(options_.maxKeypoints);
    }

    Output out;
    out.keypoints.reserve(indices.size());
    out.scores.reserve(indices.size());
    out.descriptors.create(static_cast<int>(indices.size()), D, CV_32FC1);

    const float sx = static_cast<float>(image.cols) / static_cast<float>(W);
    const float sy = static_cast<float>(image.rows) / static_cast<float>(H);

    for (size_t k = 0; k < indices.size(); ++k) {
        int i = indices[k];
        float x = readKpx(i) * sx;
        float y = readKpy(i) * sy;
        out.keypoints.emplace_back(x, y);
        out.scores.push_back(scoresData[i]);
        std::memcpy(out.descriptors.ptr<float>(static_cast<int>(k)), descData + i * D, D * sizeof(float));
    }

    return out;
}

}  // namespace vision
}  // namespace tbai
