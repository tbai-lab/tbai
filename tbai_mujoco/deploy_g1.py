#!/usr/bin/env python3

import math
import os
import sys
import dataclasses
import tkinter as tk
from tkinter import ttk

import yaml
import tyro

import tbai as tbai_python

from tbai import (
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

import tbai_descriptions


class VirtualJoystick(tk.Frame):
    def __init__(self, parent, size=200, **kwargs):
        super().__init__(parent, **kwargs)
        self.size = size
        self.center = size // 2
        self.max_distance = size // 2 - 10

        self.canvas = tk.Canvas(self, width=size, height=size, bg="lightgray")
        self.canvas.pack(padx=10, pady=10)

        self.canvas.create_oval(5, 5, size - 5, size - 5, outline="black", width=2)

        self.knob_size = 20
        self.knob_x = self.center
        self.knob_y = self.center
        self.knob = self.canvas.create_oval(
            self.knob_x - self.knob_size // 2,
            self.knob_y - self.knob_size // 2,
            self.knob_x + self.knob_size // 2,
            self.knob_y + self.knob_size // 2,
            fill="red",
            outline="darkred",
            width=2,
        )

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        self.command_callback = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

    def set_command_callback(self, callback):
        self.command_callback = callback

    def on_click(self, event):
        self.move_knob(event.x, event.y)

    def on_drag(self, event):
        self.move_knob(event.x, event.y)

    def on_release(self, event):
        self.move_knob(self.center, self.center)

    def move_knob(self, x, y):
        dx = x - self.center
        dy = y - self.center
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > self.max_distance:
            angle = math.atan2(dy, dx)
            x = self.center + self.max_distance * math.cos(angle)
            y = self.center + self.max_distance * math.sin(angle)
            dx = x - self.center
            dy = y - self.center
            distance = self.max_distance

        self.knob_x = x
        self.knob_y = y
        self.canvas.coords(
            self.knob,
            x - self.knob_size // 2,
            y - self.knob_size // 2,
            x + self.knob_size // 2,
            y + self.knob_size // 2,
        )

        self.current_x = dx / self.max_distance
        self.current_y = -dy / self.max_distance
        self.current_z = 0.0

        if self.command_callback:
            self.command_callback(self.current_x, self.current_y, self.current_z)


class G1UIController:
    def __init__(self, controller_callback=None):
        self.controller_callback = controller_callback or (lambda name: None)

        self.root = tk.Tk()
        self.root.title("G1 Virtual Joystick")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack()

        # Title
        title_label = ttk.Label(main_frame, text="G1 Humanoid Control", font=("Arial", 14, "bold"))
        title_label.pack(pady=(0, 10))

        joystick_frame = ttk.Frame(main_frame)
        joystick_frame.pack()

        # Left joystick frame (X+Y Velocity)
        left_frame = ttk.Frame(joystick_frame)
        left_frame.grid(row=0, column=0, padx=15)

        left_title = ttk.Label(left_frame, text="X+Y Velocity", font=("Arial", 12, "bold"))
        left_title.pack(pady=(0, 5))

        self.left_joystick = VirtualJoystick(left_frame, size=160)
        self.left_joystick.pack()
        self.left_joystick.set_command_callback(self.update_xy_velocity)

        # Right joystick frame (X+Yaw Velocity)
        right_frame = ttk.Frame(joystick_frame)
        right_frame.grid(row=0, column=1, padx=15)

        right_title = ttk.Label(right_frame, text="X+Yaw Velocity", font=("Arial", 12, "bold"))
        right_title.pack(pady=(0, 5))

        self.right_joystick = VirtualJoystick(right_frame, size=160)
        self.right_joystick.pack()
        self.right_joystick.set_command_callback(self.update_x_yaw_velocity)

        # Controller buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=(20, 0))

        button_title = ttk.Label(button_frame, text="Controller Mode", font=("Arial", 12, "bold"))
        button_title.pack(pady=(0, 10))

        button_grid = ttk.Frame(button_frame)
        button_grid.pack()

        # Row 1: Basic controllers
        row1_frame = ttk.Frame(button_grid)
        row1_frame.pack()

        for name, label in [("STAND", "STAND"), ("SIT", "SIT"), ("G1RLController", "WALK")]:
            ttk.Button(row1_frame, text=label, width=10, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=5
            )

        # Row 2: Mimic dance controllers
        row2_frame = ttk.Frame(button_grid)
        row2_frame.pack(pady=(10, 0))

        for name, label in [("G1MimicDance102", "DANCE"), ("G1MimicGangnam", "GANGNAM")]:
            ttk.Button(row2_frame, text=label, width=10, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=5
            )

        # Row 3: BeyondMimic controllers
        row3_frame = ttk.Frame(button_grid)
        row3_frame.pack(pady=(10, 0))
        ttk.Label(row3_frame, text="beyond mimic:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1BeyondDance", "B-DANCE"), ("G1Spinkick", "SPINKICK")]:
            ttk.Button(row3_frame, text=label, width=8, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # TWIST2 controllers
        twist_frame = ttk.Frame(button_grid)
        twist_frame.pack(pady=(10, 0))
        ttk.Label(twist_frame, text="twist2:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [
            ("G1TwistWalk1", "WALK1"),
            ("G1TwistWalk2", "WALK2"),
            ("G1TwistWalk3", "WALK3"),
            ("G1TwistWalk5", "WALK5"),
            ("G1TwistWalk7", "WALK7"),
            ("G1TwistSwing", "SWING"),
        ]:
            ttk.Button(twist_frame, text=label, width=7, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # PBHC controllers
        pbhc_frame = ttk.Frame(button_grid)
        pbhc_frame.pack(pady=(10, 0))
        ttk.Label(pbhc_frame, text="pbhc:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [
            ("G1PBHCHorseStancePunch", "PUNCH"),
            ("G1PBHCHorseStancePose", "POSE"),
            ("G1PBHCHorseStancePose2", "POSE2"),
        ]:
            ttk.Button(pbhc_frame, text=label, width=7, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # ASAP controllers - row 1
        asap_frame1 = ttk.Frame(button_grid)
        asap_frame1.pack(pady=(10, 0))
        ttk.Label(asap_frame1, text="asap:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [
            ("G1ASAPLocomotion", "LOCO"),
            ("G1ASAPCR7", "CR7"),
            ("G1ASAPAPT", "APT"),
            ("G1ASAPKobe", "KOBE"),
        ]:
            ttk.Button(asap_frame1, text=label, width=7, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # ASAP controllers - row 2
        asap_frame2 = ttk.Frame(button_grid)
        asap_frame2.pack(pady=(5, 0))
        ttk.Label(asap_frame2, text="", width=6).pack(side=tk.LEFT, padx=5)

        for name, label in [
            ("G1ASAPJumpForward1", "JUMP1"),
            ("G1ASAPJumpForward2", "JUMP2"),
            ("G1ASAPJumpForward3", "JUMP3"),
            ("G1ASAPSideJump1", "SIDE1"),
            ("G1ASAPSideJump2", "SIDE2"),
            ("G1ASAPSideJump3", "SIDE3"),
        ]:
            ttk.Button(asap_frame2, text=label, width=7, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # ASAP controllers - row 3
        asap_frame3 = ttk.Frame(button_grid)
        asap_frame3.pack(pady=(5, 0))
        ttk.Label(asap_frame3, text="", width=6).pack(side=tk.LEFT, padx=5)

        for name, label in [
            ("G1ASAPKick1", "KICK1"),
            ("G1ASAPKick2", "KICK2"),
            ("G1ASAPKick3", "KICK3"),
            ("G1ASAPLeBron1", "LEBRON1"),
            ("G1ASAPLeBron2", "LEBRON2"),
        ]:
            ttk.Button(asap_frame3, text=label, width=7, command=lambda n=name: self._on_controller(n)).pack(
                side=tk.LEFT, padx=3
            )

        # Status indicator
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(pady=(15, 0))

        self.status_label = ttk.Label(status_frame, text="Status: Ready", font=("Arial", 10))
        self.status_label.pack()

        # Velocity state
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.linear_x_scale = 0.5
        self.linear_y_scale = 0.3
        self.angular_scale = 1.4

        self.root.update_idletasks()
        self.root.after(100, self.update_gui)

    def _on_controller(self, controller_name):
        self.controller_callback(controller_name)
        self.status_label.config(text=f"Status: {controller_name}")

    def update_xy_velocity(self, x, y, z):
        self.linear_x = y * self.linear_x_scale
        self.linear_y = -x * self.linear_y_scale

    def update_x_yaw_velocity(self, x, y, z):
        self.linear_x = y * self.linear_x_scale
        self.angular_z = -x * self.angular_scale

    def get_state(self):
        return self.linear_x, self.linear_y, self.angular_z

    def update_gui(self):
        self.root.after(50, self.update_gui)

    def on_closing(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.root.quit()

    def run(self):
        self.root.mainloop()


CONFIG_FILE = os.path.join(os.path.dirname(__file__), "g1_config.yaml")

os.environ["TBAI_ROBOT_DESCRIPTION_PATH"] = str(tbai_descriptions.get_urdf_path("g1"))
os.environ["TBAI_GLOBAL_CONFIG_PATH"] = str(CONFIG_FILE)


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
            robot,
            model_path,
            paths[cfg["motion_file"]],
            motion_fps=get("motion_fps", 60.0),
            time_start=get("time_start", 0.0),
            time_end=get("time_end", -1.0),
            controller_name=ctrl_name,
        )
    elif ctrl_type == "G1BeyondMimicController":
        return tbai_python.G1BeyondMimicController(robot, model_path, ctrl_name)
    elif ctrl_type == "G1SpinkickController":
        return tbai_python.G1SpinkickController(robot, model_path, ctrl_name)
    elif ctrl_type == "G1Twist2Controller":
        return tbai_python.G1Twist2Controller(
            robot, model_path, paths[cfg["motion_file"]], get("time_start", 0.0), get("time_end", -1.0), ctrl_name
        )
    elif ctrl_type == "G1PBHCController":
        return tbai_python.G1PBHCController(
            robot, model_path, paths[cfg["motion_file"]], get("time_start", 0.0), get("time_end", -1.0), ctrl_name
        )
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
