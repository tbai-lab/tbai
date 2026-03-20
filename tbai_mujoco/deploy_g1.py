#!/usr/bin/env python3

import os
import sys
import dataclasses

import yaml
import tyro

import tbai as tbai_python

from tbai import (
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from g1_joystick import G1UIController


CONFIG_FILE = os.path.join(os.path.dirname(__file__), "g1_config.yaml")


@dataclasses.dataclass
class Args:
    net: str = "lo"
    channel: int = 1
    config: str = CONFIG_FILE


class G1ChangeControllerSubscriber(ChangeControllerSubscriber):
    def __init__(self):
        super().__init__()
        self._callback = None
        self.new_controller = None

    def setCallbackFunction(self, callback):
        self._callback = callback

    def triggerCallbacks(self):
        if self._callback is not None and self.new_controller is not None:
            self._callback(str(self.new_controller))
            self.new_controller = None

    def request(self, controller_name):
        self.new_controller = controller_name


class G1ReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: G1UIController, ramped_velocity: float = 5.0):
        super().__init__()
        self.ui_controller = ui_controller
        self.ramped_velocity = ramped_velocity
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def getReferenceVelocity(self, time, dt):
        desired_x = self.ui_controller.linear_x
        desired_y = self.ui_controller.linear_y
        desired_yaw = self.ui_controller.angular_z

        max_step = self.ramped_velocity * dt

        sign = lambda v: 1.0 if v >= 0.0 else -1.0
        self.current_x += sign(desired_x - self.current_x) * min(abs(desired_x - self.current_x), max_step)
        self.current_y += sign(desired_y - self.current_y) * min(abs(desired_y - self.current_y), max_step)
        self.current_yaw += sign(desired_yaw - self.current_yaw) * min(abs(desired_yaw - self.current_yaw), max_step)

        ref = ReferenceVelocity()
        ref.velocity_x = self.current_x
        ref.velocity_y = self.current_y
        ref.yaw_rate = self.current_yaw
        return ref


def download(repo, filename):
    print(f"  {filename}...")
    return tbai_python.download_from_huggingface(repo, filename)


def load_controllers(config, robot, ref_vel_gen, central_controller):
    hf_repo = config["hf_repo"]
    downloaded = {}

    def get_model(filename):
        if filename not in downloaded:
            downloaded[filename] = download(hf_repo, filename)
        return downloaded[filename]

    print("Downloading models...")

    for name, cfg in config["controllers"].items():
        ctrl_type = cfg["type"]
        model_path = get_model(cfg["model"])
        ctrl_name = cfg.get("controller_name", name)

        if ctrl_type == "G1RLController":
            ctrl = tbai_python.G1RLController(robot, ref_vel_gen, model_path)

        elif ctrl_type == "G1ASAPController":
            ctrl = tbai_python.G1ASAPController(robot, ref_vel_gen, model_path, ctrl_name)

        elif ctrl_type == "G1MimicController":
            motion_path = get_model(cfg["motion_file"])
            ctrl = tbai_python.G1MimicController(
                robot, model_path, motion_path,
                motion_fps=cfg.get("motion_fps", 60.0),
                time_start=cfg.get("time_start", 0.0),
                time_end=cfg.get("time_end", -1.0),
                controller_name=ctrl_name)

        elif ctrl_type == "G1BeyondMimicController":
            ctrl = tbai_python.G1BeyondMimicController(robot, model_path, ctrl_name)

        elif ctrl_type == "G1SpinkickController":
            ctrl = tbai_python.G1SpinkickController(robot, model_path, ctrl_name)

        elif ctrl_type == "G1Twist2Controller":
            motion_path = get_model(cfg["motion_file"])
            ctrl = tbai_python.G1Twist2Controller(
                robot, model_path, motion_path,
                cfg.get("time_start", 0.0), cfg.get("time_end", -1.0), ctrl_name)

        elif ctrl_type == "G1PBHCController":
            motion_path = get_model(cfg["motion_file"])
            ctrl = tbai_python.G1PBHCController(
                robot, model_path, motion_path,
                cfg.get("time_start", 0.0), cfg.get("time_end", -1.0), ctrl_name)

        elif ctrl_type == "G1ASAPMimicController":
            ctrl = tbai_python.G1ASAPMimicController(
                robot, model_path, cfg["motion_length"], ctrl_name)

        else:
            print(f"  Warning: unknown controller type '{ctrl_type}' for '{name}', skipping")
            continue

        central_controller.add_controller(ctrl)
        print(f"  Loaded {ctrl_name} ({ctrl_type})")


def main():
    args = tyro.cli(Args)

    if not tbai_python.HAS_DEPLOY_G1:
        print("Error: tbai_python was built without G1 support (TBAI_BUILD_DEPLOY_G1=OFF)")
        sys.exit(1)

    config_path = os.path.abspath(args.config)
    os.environ["TBAI_GLOBAL_CONFIG_PATH"] = config_path

    with open(config_path) as f:
        config = yaml.safe_load(f)

    # Initialize G1 robot
    robot_args = tbai_python.G1RobotInterfaceArgs()
    robot_args.set_network_interface(args.net)
    robot_args.set_unitree_channel(args.channel)
    robot_args.set_channel_init(True)

    print(f"Connecting to G1 on {args.net} (channel {args.channel})...")
    robot = tbai_python.G1RobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.waitTillInitialized()
    print("Robot initialized.")

    controller_sub = G1ChangeControllerSubscriber()

    ui_controller = G1UIController(
        controller_callback=lambda name: controller_sub.request(name),
    )
    ref_vel_gen = G1ReferenceVelocityGenerator(ui_controller)

    tbai_python.write_init_time()

    central_controller = tbai_python.CentralController.create(robot, controller_sub)

    # Static controller (stand/sit)
    static_ctrl = tbai_python.StaticController(robot, None)
    central_controller.add_controller(static_ctrl)

    # Load all controllers from config
    load_controllers(config, robot, ref_vel_gen, central_controller)

    print("All controllers loaded. Starting...")

    central_controller.startThread()

    try:
        ui_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping controller...")
        central_controller.stopThread()
        print("Done.")


if __name__ == "__main__":
    main()
