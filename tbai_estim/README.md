## tbai_estim

State estimators for tbai. Currently:

- **InEKF** — Invariant Extended Kalman Filter for legged-robot proprioceptive state
  (IMU + joint kinematics + foot contacts). Based on
  [invariant-ekf](https://github.com/RossHartley/invariant-ekf) by Ross Hartley.

- **Visual landmarks (optional)** — feeds 3D landmarks derived from images +
  point clouds into the InEKF as `CorrectLandmarks` measurements. Built only
  when `-DTBAI_BUILD_VO=ON` is passed to CMake.

### Visual-odometry subsystem

Built when `TBAI_BUILD_VO=ON`. Pulls in OpenCV (already a workspace dep) and
ONNX Runtime (the same one fetched by `tbai_deploy_g1`).

Pipeline:

```
cv::Mat image  ─►  SuperPointExtractor  ─►  keypoints + descriptors
                         │
PointCloud2    ─►  PointCloudIndexer  ─►  3D point lookup at (u, v)
                         │
                         ▼
              VisualLandmarkTracker
                  (descriptor matching,
                   persistent landmark ids)
                         │
                         ▼
            std::vector<inekf::Landmark>
                         │
                         ▼
   InEKFEstimator::correctVisualLandmarks(...)
```

Persistent landmark ids matter: `inekf::InEKF::CorrectLandmarks` only does a
measurement update when an id matches a previously-tracked landmark; new ids
just augment the state. The tracker handles this via L2 + Lowe ratio
descriptor matching against the previous frame.

#### One-time setup

Download the SuperPoint ONNX model (about 5 MB):

```bash
python tbai_estim/scripts/download_superpoint_onnx.py --output ~/.tbai/superpoint.onnx
```

The script pulls from the
[LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) release page,
the same source kornia's `OnnxLightGlue` uses.

The pretrained SuperPoint weights are released under MagicLeap's
**research / non-commercial** license — see
[magicleap/SuperPointPretrainedNetwork](https://github.com/magicleap/SuperPointPretrainedNetwork).
Swap in a different feature extractor with compatible ONNX I/O if your use is
commercial.

#### C++ usage

```cpp
#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_estim/vision/PointCloudIndexer.hpp>
#include <tbai_estim/vision/SuperPointExtractor.hpp>
#include <tbai_estim/vision/VisualLandmarkTracker.hpp>

tbai::inekf::InEKFEstimator filter(footNames, urdf);

tbai::vision::SuperPointExtractor::Options spOpts;
spOpts.inputWidth = 640;
spOpts.inputHeight = 480;
spOpts.maxKeypoints = 512;
tbai::vision::SuperPointExtractor extractor("/path/to/superpoint.onnx", spOpts);

tbai::vision::VisualLandmarkTracker tracker;

while (true) {
    cv::Mat image = ...;                      // CV_8UC1 or CV_8UC3
    // organized cloud aligned to the image; (u, v) maps to a 3D point in the
    // body / robot frame. (Convert from PointCloud2 once at the call site.)
    tbai::vision::PointCloudIndexer cloud(cloudData, height, width, rowStride, pointStride);

    auto landmarks = tracker.processFrame(extractor, image, cloud);
    filter.correctVisualLandmarks(landmarks);
}
```

#### PointCloud2 → indexer adapter

`PointCloudIndexer` deliberately depends only on Eigen, not on `tbai_sdk`
or any ROS-style message type, so `tbai_estim` stays leaf-level. Build the
indexer at the call site by passing the raw fields of a `PointCloud2`:

```cpp
robot_msgs::PointCloud2 pc = ...;
// Find the byte offset of the x field once at startup. The indexer assumes
// y, z follow as float32 immediately after x (the common XYZ layout).
int xOff = 0;
for (const auto &f : pc.fields) {
    if (f.name == "x") { xOff = f.offset; break; }
}
tbai::vision::PointCloudIndexer cloud(pc.data.data(), pc.height, pc.width,
                                      pc.row_step, pc.point_step, xOff);
```

The cloud is assumed to be in the body / robot frame (matching the convention
used by `AnymalCRobotInterface` for `rt/pointcloud/front_lower`). If your
sensor publishes in the camera frame, transform it before calling the indexer.

#### Build

```bash
cmake -B build -S. -DTBAI_BUILD_VO=ON -DTBAI_BUILD_TESTS=ON
cmake --build build --parallel
TBAI_SUPERPOINT_ONNX=~/.tbai/superpoint.onnx ctest --test-dir build --output-on-failure
```

Tests gated on the SuperPoint ONNX file (`test_superpoint_extractor`,
`test_vo_end_to_end`) skip when `TBAI_SUPERPOINT_ONNX` is unset.

## Credits

- InEKF implementation based on [invariant-ekf](https://github.com/RossHartley/invariant-ekf) by Ross Hartley.
- SuperPoint weights from MagicLeap's
  [SuperPointPretrainedNetwork](https://github.com/magicleap/SuperPointPretrainedNetwork)
  (non-commercial research license), distributed in ONNX form by
  [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX).
