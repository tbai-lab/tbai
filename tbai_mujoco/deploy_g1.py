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
    ground_truth: bool = False
    view_image_stream: bool = False
    image_topic: str = "rt/camera/image"


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


def _download_all(hf_repo, controllers):
    """Download all model/motion files in parallel, return {filename: path}."""
    import threading

    files = set()
    for cfg in controllers.values():
        files.add(cfg["model"])
        if "motion_file" in cfg:
            files.add(cfg["motion_file"])

    paths = {}

    def dl(f):
        paths[f] = download(hf_repo, f)

    threads = [threading.Thread(target=dl, args=(f,)) for f in files]
    _ = [t.start() for t in threads]
    _ = [t.join() for t in threads]
    return paths


def _make_controller(cfg, robot, ref_vel_gen, paths):
    """Construct a single controller from config. Returns (controller, name) or None."""
    ctrl_type = cfg["type"]
    model_path = paths[cfg["model"]]
    ctrl_name = cfg.get("controller_name")
    get = cfg.get

    if ctrl_type == "G1RLController":
        return tbai_python.G1RLController(robot, ref_vel_gen, model_path)
    elif ctrl_type == "G1ASAPController":
        return tbai_python.G1ASAPController(robot, ref_vel_gen, model_path, ctrl_name)
    elif ctrl_type == "G1MimicController":
        return tbai_python.G1MimicController(
            robot, model_path, paths[cfg["motion_file"]],
            motion_fps=get("motion_fps", 60.0), time_start=get("time_start", 0.0),
            time_end=get("time_end", -1.0), controller_name=ctrl_name)
    elif ctrl_type == "G1BeyondMimicController":
        return tbai_python.G1BeyondMimicController(robot, model_path, ctrl_name)
    elif ctrl_type == "G1SpinkickController":
        return tbai_python.G1SpinkickController(robot, model_path, ctrl_name)
    elif ctrl_type == "G1Twist2Controller":
        return tbai_python.G1Twist2Controller(
            robot, model_path, paths[cfg["motion_file"]],
            get("time_start", 0.0), get("time_end", -1.0), ctrl_name)
    elif ctrl_type == "G1PBHCController":
        return tbai_python.G1PBHCController(
            robot, model_path, paths[cfg["motion_file"]],
            get("time_start", 0.0), get("time_end", -1.0), ctrl_name)
    elif ctrl_type == "G1ASAPMimicController":
        return tbai_python.G1ASAPMimicController(robot, model_path, cfg["motion_length"], ctrl_name)
    else:
        print(f"  Warning: unknown controller type '{ctrl_type}', skipping")
        return None


def load_controllers(config, robot, ref_vel_gen, central_controller):
    print("Downloading models...")
    paths = _download_all(config["hf_repo"], config["controllers"])

    for name, cfg in config["controllers"].items():
        ctrl = _make_controller(cfg, robot, ref_vel_gen, paths)
        assert ctrl is not None, f"Failed to load controller {name}"
        central_controller.add_controller(ctrl)
        print(f"  Loaded {cfg.get('controller_name', name)} ({cfg['type']})")


class ImageViewer:
    def __init__(self, topic: str):
        from tbai_sdk.subscriber import PollingSubscriber
        from tbai_sdk.messages.robot_msgs import ImgFrame
        import matplotlib.pyplot as plt
        import numpy as np

        self._sub = PollingSubscriber(ImgFrame, topic)
        self._plt = plt
        self._np = np
        self._fig, self._ax = plt.subplots()
        self._img_handle = None
        self._ax.set_axis_off()
        self._fig.tight_layout()
        print(f"ImageViewer: subscribed to {topic}")

    def update(self):
        frame = self._sub.take()
        if frame is None:
            return
        np = self._np
        if frame.encoding in ("rgb8", "bgr8"):
            arr = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 3)
            if frame.encoding == "bgr8":
                arr = arr[:, :, ::-1]
        elif frame.encoding == "mono8":
            arr = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width)
        else:
            return
        if self._img_handle is None:
            self._img_handle = self._ax.imshow(arr)
        else:
            self._img_handle.set_data(arr)
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def start_nonblocking(self):
        self._plt.ion()
        self._plt.show(block=False)

    def close(self):
        self._plt.close(self._fig)


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
    if args.ground_truth:
        robot_args.set_enable_state_estim(False)

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

    # Image viewer
    image_viewer = None
    if args.view_image_stream:
        image_viewer = ImageViewer(args.image_topic)
        image_viewer.start_nonblocking()

    print("All controllers loaded. Starting...")

    central_controller.startThread()

    try:
        if image_viewer:
            def poll_image():
                image_viewer.update()
                ui_controller.root.after(33, poll_image)
            ui_controller.root.after(33, poll_image)
        ui_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        if image_viewer:
            image_viewer.close()
        print("Stopping controller...")
        central_controller.stopThread()
        print("Done.")


if __name__ == "__main__":
    main()
