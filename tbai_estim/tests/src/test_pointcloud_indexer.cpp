#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <tbai_estim/vision/PointCloudIndexer.hpp>

namespace {

// Build a tightly-packed organized cloud (h x w) with each pixel storing
// (x = col, y = row, z = col + row) as float32.
std::vector<std::uint8_t> makePackedXYZ(int height, int width) {
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(height * width * 3 * sizeof(float)));
    auto *p = reinterpret_cast<float *>(buf.data());
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            int idx = (v * width + u) * 3;
            p[idx + 0] = static_cast<float>(u);
            p[idx + 1] = static_cast<float>(v);
            p[idx + 2] = static_cast<float>(u + v);
        }
    }
    return buf;
}

}  // namespace

TEST(PointCloudIndexer, LooksUpInBounds) {
    const int H = 4, W = 5;
    auto buf = makePackedXYZ(H, W);
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, /*rowStride=*/W * 3 * sizeof(float),
                                        /*pointStride=*/3 * sizeof(float));
    auto p = idx.atPixel(3, 2);
    ASSERT_TRUE(p.has_value());
    EXPECT_DOUBLE_EQ((*p)[0], 3.0);
    EXPECT_DOUBLE_EQ((*p)[1], 2.0);
    EXPECT_DOUBLE_EQ((*p)[2], 5.0);
}

TEST(PointCloudIndexer, OutOfBoundsReturnsNullopt) {
    const int H = 4, W = 5;
    auto buf = makePackedXYZ(H, W);
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));
    EXPECT_FALSE(idx.atPixel(-1, 0).has_value());
    EXPECT_FALSE(idx.atPixel(0, -1).has_value());
    EXPECT_FALSE(idx.atPixel(W, 0).has_value());
    EXPECT_FALSE(idx.atPixel(0, H).has_value());
}

TEST(PointCloudIndexer, RejectsNaNPoints) {
    const int H = 2, W = 2;
    auto buf = makePackedXYZ(H, W);
    auto *p = reinterpret_cast<float *>(buf.data());
    // Pixel (1, 0): index = (0 * 2 + 1) * 3 = 3
    p[3] = std::numeric_limits<float>::quiet_NaN();
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));
    EXPECT_FALSE(idx.atPixel(1, 0).has_value());
    EXPECT_TRUE(idx.atPixel(0, 0).has_value());
}

TEST(PointCloudIndexer, RejectsBeyondMaxRange) {
    const int H = 1, W = 2;
    std::vector<std::uint8_t> buf(2 * 3 * sizeof(float));
    auto *p = reinterpret_cast<float *>(buf.data());
    p[0] = 1.0f; p[1] = 0.0f; p[2] = 0.0f;     // norm 1
    p[3] = 10.0f; p[4] = 0.0f; p[5] = 0.0f;    // norm 10
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float),
                                        /*xOffset=*/0, /*maxRange=*/5.0);
    EXPECT_TRUE(idx.atPixel(0, 0).has_value());
    EXPECT_FALSE(idx.atPixel(1, 0).has_value());
}

TEST(PointCloudIndexer, HonoursPointAndRowStrides) {
    // PointCloud2-style: point_step = 16 (xyz + padding), row_step = width * point_step.
    const int H = 2, W = 3;
    const int pointStride = 16;
    const int rowStride = W * pointStride;
    std::vector<std::uint8_t> buf(H * rowStride, 0);
    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            float xyz[3] = {static_cast<float>(u), static_cast<float>(v), static_cast<float>(u * v)};
            std::memcpy(buf.data() + v * rowStride + u * pointStride, xyz, sizeof(xyz));
        }
    }
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, rowStride, pointStride);
    auto p = idx.atPixel(2, 1);
    ASSERT_TRUE(p.has_value());
    EXPECT_DOUBLE_EQ((*p)[0], 2.0);
    EXPECT_DOUBLE_EQ((*p)[1], 1.0);
    EXPECT_DOUBLE_EQ((*p)[2], 2.0);
}

TEST(PointCloudIndexer, FloatingPointPixelRoundsToNearest) {
    const int H = 4, W = 4;
    auto buf = makePackedXYZ(H, W);
    tbai::vision::PointCloudIndexer idx(buf.data(), H, W, W * 3 * sizeof(float), 3 * sizeof(float));
    // 1.6 rounds to 2; 2.4 rounds to 2.
    auto p = idx.at(1.6, 2.4);
    ASSERT_TRUE(p.has_value());
    EXPECT_DOUBLE_EQ((*p)[0], 2.0);
    EXPECT_DOUBLE_EQ((*p)[1], 2.0);
}
