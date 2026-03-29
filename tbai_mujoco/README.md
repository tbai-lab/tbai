# tbai_mujoco

## Install dependencies

```bash
pip3 install -r ./requirements.txt
```

## Run
```bash
## terminal 1 - sim (pick your robot) - ensure tbai_mujoco is properly installed!
tbai_mujoco $(python3 -m tbai_mujoco_descriptions.print_config_path go2)
tbai_mujoco $(python3 -m tbai_mujoco_descriptions.print_config_path g1)

## terminal 2 - controller (pick your robot)
python3 ./deploy_g1.py
python3 ./deploy_go2.py

```
