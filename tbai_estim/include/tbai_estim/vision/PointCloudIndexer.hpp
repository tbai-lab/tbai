#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace tbai {
namespace vision {

// Pixel-indexed lookup of 3D points in an organized point grid (height x width).
//
// The indexer does not own the data — caller is responsible for keeping the
// underlying buffer alive. The buffer is interpreted as `height` rows of
// `width` points; consecutive rows are `rowStride` bytes apart and consecutive
// points are `pointStride` bytes apart. The `xOffset` argument is the byte
// offset of the x coordinate inside one point; y/z follow as float32 directly
// after x. (This matches the typical XYZ field layout of an organized
// PointCloud2 with field_step = sizeof(float).)
//
// A point is considered invalid when its x coordinate is NaN or its norm
// exceeds `maxRange` (set negative to disable the range check).
class PointCloudIndexer {
   public:
    PointCloudIndexer(const std::uint8_t *data, int height, int width, int rowStride, int pointStride, int xOffset = 0,
                      double maxRange = -1.0);

    int height() const { return height_; }
    int width() const { return width_; }

    // Sample the 3D point at the given pixel using nearest-neighbour rounding.
    // Returns std::nullopt for out-of-bounds pixels or invalid points.
    std::optional<Eigen::Vector3d> at(double u, double v) const;

    // Same as `at` but takes integer pixel coordinates (no rounding).
    std::optional<Eigen::Vector3d> atPixel(int u, int v) const;

   private:
    const std::uint8_t *data_;
    int height_;
    int width_;
    int rowStride_;
    int pointStride_;
    int xOffset_;
    double maxRangeSq_;
    bool checkRange_;
};

}  // namespace vision
}  // namespace tbai
