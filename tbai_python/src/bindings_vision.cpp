#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#ifdef TBAI_HAS_VO
#include <opencv2/core.hpp>

#include <tbai_estim/inekf/Observations.hpp>
#include <tbai_estim/vision/PointCloudIndexer.hpp>
#include <tbai_estim/vision/SuperPointExtractor.hpp>
#include <tbai_estim/vision/VisualLandmarkTracker.hpp>
#endif

namespace nb = nanobind;

void bind_vision(nb::module_ &m) {
    m.attr("HAS_VO") =
#ifdef TBAI_HAS_VO
        true;
#else
        false;
#endif

#ifdef TBAI_HAS_VO
    auto vo = m.def_submodule("vision", "Visual-odometry primitives (SuperPoint, descriptor tracker, depth lookup).");

    using Landmark = ::inekf::Landmark;
    nb::class_<Landmark>(vo, "Landmark")
        .def(nb::init<int, Eigen::Vector3d, Eigen::Matrix3d>(), nb::arg("id"), nb::arg("position"),
             nb::arg("covariance"))
        .def_rw("id", &Landmark::id)
        .def_rw("position", &Landmark::position)
        .def_rw("covariance", &Landmark::covariance);

    using Idx = tbai::vision::PointCloudIndexer;
    nb::class_<Idx>(vo, "PointCloudIndexer", "Pixel-indexed lookup of 3D points in an organized point grid.")
        .def("__init__",
             [](Idx *self,
                nb::ndarray<const std::uint8_t, nb::c_contig> data, int height, int width, int rowStride,
                int pointStride, int xOffset, double maxRange) {
                 new (self) Idx(data.data(), height, width, rowStride, pointStride, xOffset, maxRange);
             },
             nb::arg("data"), nb::arg("height"), nb::arg("width"), nb::arg("row_stride"), nb::arg("point_stride"),
             nb::arg("x_offset") = 0, nb::arg("max_range") = -1.0,
             nb::keep_alive<1, 2>(),  // keep `data` alive as long as the indexer
             "Build an indexer over a contiguous (height x width x point_step) byte buffer. "
             "x_offset is the byte offset of the X coordinate inside one point; y, z follow as float32.")
        .def_prop_ro("height", &Idx::height)
        .def_prop_ro("width", &Idx::width)
        .def("at_pixel", &Idx::atPixel, nb::arg("u"), nb::arg("v"),
             "Sample the 3D point at the integer pixel (u, v). Returns None for OOB / NaN / out-of-range.");

    using SP = tbai::vision::SuperPointExtractor;
    nb::class_<SP::Options>(vo, "SuperPointOptions")
        .def(nb::init<>())
        .def_rw("input_width", &SP::Options::inputWidth)
        .def_rw("input_height", &SP::Options::inputHeight)
        .def_rw("score_threshold", &SP::Options::scoreThreshold)
        .def_rw("max_keypoints", &SP::Options::maxKeypoints)
        .def_rw("descriptor_dim", &SP::Options::descriptorDim)
        .def_rw("input_name", &SP::Options::inputName)
        .def_rw("keypoints_output_name", &SP::Options::keypointsOutputName)
        .def_rw("scores_output_name", &SP::Options::scoresOutputName)
        .def_rw("descriptors_output_name", &SP::Options::descriptorsOutputName)
        .def_rw("intra_op_threads", &SP::Options::intraOpThreads);

    nb::class_<SP>(vo, "SuperPointExtractor", "ONNX-backed SuperPoint detector + descriptor extractor.")
        .def(nb::init<const std::string &>(), nb::arg("onnx_path"))
        .def(nb::init<const std::string &, const SP::Options &>(), nb::arg("onnx_path"), nb::arg("options"))
        .def(
            "detect",
            [](SP &self,
               nb::ndarray<const std::uint8_t, nb::c_contig> image)
                -> std::tuple<nb::ndarray<nb::numpy, float>, nb::ndarray<nb::numpy, float>,
                              nb::ndarray<nb::numpy, float>> {
                if (image.ndim() != 2 && image.ndim() != 3) {
                    throw std::invalid_argument("SuperPointExtractor.detect: image must be 2D (H,W) or 3D (H,W,3)");
                }
                int height = static_cast<int>(image.shape(0));
                int width = static_cast<int>(image.shape(1));
                int channels = (image.ndim() == 3) ? static_cast<int>(image.shape(2)) : 1;
                int cvType = (channels == 1) ? CV_8UC1 : CV_8UC3;
                cv::Mat cvImage(height, width, cvType, const_cast<std::uint8_t *>(image.data()));

                SP::Output out;
                {
                    nb::gil_scoped_release release;
                    out = self.detect(cvImage);
                }

                const size_t N = out.keypoints.size();
                const int D = self.descriptorDim();

                // Allocate Python-owned output arrays.
                auto *kpsBuf = new float[N * 2];
                auto *scoresBuf = new float[N];
                auto *descBuf = new float[N * D];
                for (size_t i = 0; i < N; ++i) {
                    kpsBuf[i * 2 + 0] = out.keypoints[i].x;
                    kpsBuf[i * 2 + 1] = out.keypoints[i].y;
                    scoresBuf[i] = out.scores[i];
                }
                if (N > 0) {
                    std::memcpy(descBuf, out.descriptors.ptr<float>(0), N * D * sizeof(float));
                }
                nb::capsule kpsOwner(kpsBuf, [](void *p) noexcept { delete[] static_cast<float *>(p); });
                nb::capsule scoresOwner(scoresBuf, [](void *p) noexcept { delete[] static_cast<float *>(p); });
                nb::capsule descOwner(descBuf, [](void *p) noexcept { delete[] static_cast<float *>(p); });

                size_t kpsShape[2] = {N, 2};
                size_t scoresShape[1] = {N};
                size_t descShape[2] = {N, static_cast<size_t>(D)};
                nb::ndarray<nb::numpy, float> kpsNp(kpsBuf, 2, kpsShape, kpsOwner);
                nb::ndarray<nb::numpy, float> scoresNp(scoresBuf, 1, scoresShape, scoresOwner);
                nb::ndarray<nb::numpy, float> descNp(descBuf, 2, descShape, descOwner);
                return std::make_tuple(std::move(kpsNp), std::move(scoresNp), std::move(descNp));
            },
            nb::arg("image"),
            "Run the extractor on a uint8 grayscale (H,W) or BGR (H,W,3) image. "
            "Returns (keypoints[N,2], scores[N], descriptors[N,D]) as numpy float32 arrays.")
        .def_prop_ro("input_width", &SP::inputWidth)
        .def_prop_ro("input_height", &SP::inputHeight)
        .def_prop_ro("descriptor_dim", &SP::descriptorDim);

    using Tracker = tbai::vision::VisualLandmarkTracker;
    nb::class_<Tracker::Options>(vo, "TrackerOptions")
        .def(nb::init<>())
        .def_rw("ratio_threshold", &Tracker::Options::ratioThreshold)
        .def_rw("max_descriptor_distance", &Tracker::Options::maxDescriptorDistance)
        .def_rw("landmark_variance", &Tracker::Options::landmarkVariance)
        .def_rw("first_id", &Tracker::Options::firstId);

    nb::class_<Tracker>(vo, "VisualLandmarkTracker",
                        "Frame-to-frame descriptor matcher producing inekf::Landmark observations.")
        .def(nb::init<>())
        .def(nb::init<const Tracker::Options &>(), nb::arg("options"))
        .def(
            "process_frame",
            [](Tracker &self,
               nb::ndarray<const float, nb::c_contig> keypoints,
               nb::ndarray<const float, nb::c_contig> descriptors,
               const Idx &cloud) -> ::inekf::vectorLandmarks {
                if (keypoints.ndim() != 2 || keypoints.shape(1) != 2) {
                    throw std::invalid_argument("process_frame: keypoints must be (N,2) float32");
                }
                if (descriptors.ndim() != 2) {
                    throw std::invalid_argument("process_frame: descriptors must be (N,D) float32");
                }
                if (keypoints.shape(0) != descriptors.shape(0)) {
                    throw std::invalid_argument("process_frame: keypoints/descriptors row count mismatch");
                }
                const int N = static_cast<int>(keypoints.shape(0));
                const int D = static_cast<int>(descriptors.shape(1));

                std::vector<cv::Point2f> kps;
                kps.reserve(N);
                for (int i = 0; i < N; ++i) {
                    kps.emplace_back(keypoints.data()[i * 2 + 0], keypoints.data()[i * 2 + 1]);
                }
                cv::Mat desc(N, D, CV_32FC1, const_cast<float *>(descriptors.data()));
                nb::gil_scoped_release release;
                return self.processFrame(kps, desc, cloud);
            },
            nb::arg("keypoints"), nb::arg("descriptors"), nb::arg("cloud"),
            "Process pre-extracted features against the previous frame; returns the list of landmarks.")
        .def("reset", &Tracker::reset)
        .def_prop_ro("tracked_count", &Tracker::trackedCount)
        .def_prop_ro("next_id", &Tracker::nextId);
#endif  // TBAI_HAS_VO
}
