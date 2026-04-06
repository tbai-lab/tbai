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
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 10
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
None of this would have been possible were it not for the many amazing open-source projects here on Github. Please, navigate to [CREDITS](./CREDITS.md) to see a non-exhaustive list of repositories and links most inspiration was drawn from that have been instrumental during tbai's development.

Thank you all 🤗