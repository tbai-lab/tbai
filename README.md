# Towards Better Athletic Intelligence

[![Tests](https://github.com/tbai-lab/tbai/actions/workflows/tests.yml/badge.svg)](https://github.com/tbai-lab/tbai/actions/workflows/tests.yml)

This repository contains implementations of core algorithms used in the `tbai` ecosystems. For deployment and use with specific robotics frameworks, including ROS and ROS2, thin wrapper repositories are available: [tbai_ros](https://github.com/lnotspotl/tbai_ros) and [tbai_ros2](https://github.com/tbai-lab/tbai_ros2)

<div align="center">

<img src="https://github.com/user-attachments/assets/5351d23c-59fd-47d2-8e8a-3f459b403339" width="60%" alt="demo"/>

</div>

- [**tbai_ros**](https://github.com/tbai-lab/tbai_ros) - a ROS-noetic wrapper around tbai, uses [pixi](https://pixi.sh) for dependency management, so no worries that ROS is past its end of life :) - works on many Ubuntu releases, including Ubuntu 24.04
- [**tbai_ros2**](https://github.com/tbai-lab/tbai_ros2) - a ROS2-jazzy wrapper around tbai, uses [pixi](https://pixi.sh) for dependency management - works on many Ubuntu releases, including Ubuntu 20.04, Ubuntu 22.04 and Ubuntu 24.04

#### Install `tbai`
```bash
pixi install && pixi shell
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10
```

### Tbai as a framework

<div align="center">
  
<img src="https://github.com/user-attachments/assets/41d7fb3b-afdb-46a5-bd05-25ceaaef0682" width="60%" alt="tbai"/>

</div>


#### Environment Variables used throughout `tbai`

| Environment Variable | Type | Default | Description | Usage |
|---------------------|------|---------|-------------|-------|
| **`TBAI_LOG_LEVEL`** | `string` | `"info"` | Logging level | Log verbosity. Values: `"trace"`, `"debug"`, `"info"`, `"warn"`, `"error"`, `"critical"` |
| **`TBAI_LOG_FOLDER`** | `string` | `""` (empty) | Log file directory. No file logging if empty | Logs saved here if set |
| **`TBAI_LOG_TO_CONSOLE`** | `bool` | `true` | Log to console | Show logs in terminal |
| **`TBAI_GLOBAL_CONFIG_PATH`** | `string` | **Required** | Global config file path | Main YAML config file |
| **`TBAI_ROBOT_DESCRIPTION_PATH`** | `string` | **Required** | Robot URDF path | Robot model file |
| **`TBAI_CACHE_DIR`** | `string` | `"/tmp/tbai_hf_cache"` | Model cache directory | For downloaded models |


### Credits
This project stands on the shoulders of giants.
None of this would have been possible were it not for many amazing open-source projects.
Here are a couple that most inspiration was drawn from and that were instrumental during the development:

- https://github.com/leggedrobotics/ocs2
- https://github.com/qiayuanl/legged_control
- https://github.com/leggedrobotics/legged_gym
- https://github.com/leggedrobotics/rsl_rl
- https://github.com/ANYbotics/elevation_mapping
- https://github.com/leggedrobotics/elevation_mapping_cupy
- https://github.com/bernhardpg/quadruped_locomotion
- https://github.com/stack-of-tasks/pinocchio
- https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- https://github.com/mayataka/robotoc
- https://github.com/mayataka/legged_state_estimator
- https://github.com/RossHartley/invariant-ekf
- https://github.com/dfki-ric-underactuated-lab/dfki-quad
- https://github.com/iit-DLSLab/muse
- https://github.com/zeonsunlightyu/LocomotionWithNP3O
- https://github.com/letaicodeit/DreamWaQ_Go2W
- https://cmp.felk.cvut.cz/~peckama2/ (supervision, access to a DGX station)
- http://www.michaelsebek.cz/cs
- https://github.com/HansZ8/RoboJuDo
- https://github.com/HybridRobotics/motion_tracking_controller
- https://hier-robotics.github.io/ (professor Jaemin Lee, supervision, access to Go2s)
- hundreds of others ...

Thank you all 🤗