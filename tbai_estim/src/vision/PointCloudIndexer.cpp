#include <tbai_estim/vision/PointCloudIndexer.hpp>

#include <cmath>
#include <cstring>

namespace tbai {
namespace vision {

PointCloudIndexer::PointCloudIndexer(const std::uint8_t *data, int height, int width, int rowStride, int pointStride,
                                     int xOffset, double maxRange)
    : data_(data),
      height_(height),
      width_(width),
      rowStride_(rowStride),
      pointStride_(pointStride),
      xOffset_(xOffset),
      maxRangeSq_(maxRange * maxRange),
      checkRange_(maxRange > 0.0) {}

std::optional<Eigen::Vector3d> PointCloudIndexer::atPixel(int u, int v) const {
    if (u < 0 || u >= width_ || v < 0 || v >= height_) {
        return std::nullopt;
    }
    const std::uint8_t *p = data_ + static_cast<std::ptrdiff_t>(v) * rowStride_
                            + static_cast<std::ptrdiff_t>(u) * pointStride_ + xOffset_;
    float xyz[3];
    std::memcpy(xyz, p, sizeof(xyz));
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
        return std::nullopt;
    }
    Eigen::Vector3d point(xyz[0], xyz[1], xyz[2]);
    if (checkRange_ && point.squaredNorm() > maxRangeSq_) {
        return std::nullopt;
    }
    return point;
}

std::optional<Eigen::Vector3d> PointCloudIndexer::at(double u, double v) const {
    int ui = static_cast<int>(std::lround(u));
    int vi = static_cast<int>(std::lround(v));
    return atPixel(ui, vi);
}

}  // namespace vision
}  // namespace tbai
