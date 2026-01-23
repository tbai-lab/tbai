# tbai - Towards Better Athletic Intelligence

Welcome to the tbai documentation!

<div align="center">
  <img src="https://github.com/user-attachments/assets/5351d23c-59fd-47d2-8e8a-3f459b403339" width="60%" alt="demo"/>
</div>

#### Tbai as a framework

<div align="center">
  <img src="https://github.com/user-attachments/assets/41d7fb3b-afdb-46a5-bd05-25ceaaef0682" width="60%" alt="tbai"/>
</div>

<div style="font-size: 85%;">

#### Environment Variables

```{list-table}
:header-rows: 1
:widths: 30 20 50

* - Variable
  - Default
  - Description
* - ``TBAI_GLOBAL_CONFIG_PATH``
  - ``required``
  - ``Path to the global YAML configuration file``
* - ``TBAI_ROBOT_DESCRIPTION_PATH``
  - ``required``
  - ``Path to the robot URDF file``
* - ``TBAI_LOG_LEVEL``
  - ``"info"``
  - ``Logging level (trace, debug, info, warn, error, critical)``
* - ``TBAI_LOG_FOLDER``
  - ``""``
  - ``Folder for log files (empty = no file logging)``
* - ``TBAI_LOG_TO_CONSOLE``
  - ``true``
  - ``Whether to output logs to console``
* - ``TBAI_CACHE_DIR``
  - ``/tmp/tbai_hf_cache``
  - ``Cache directory for downloaded models``
```

</div>

```{toctree}
:maxdepth: 1

example_rst
example_md
```
