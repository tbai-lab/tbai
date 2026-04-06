#!/usr/bin/env python3

import math
import os
import sys
import dataclasses
import tkinter as tk
from tkinter import ttk

import numpy as np
import tyro

import tbai as tbai_python
from tbai import ChangeControllerSubscriber

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

        self.current_x = 0.0
        self.current_y = 0.0

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


class EEJoystickUI:
    """Virtual joystick UI for controlling the Franka end-effector position."""

    def __init__(
        self,
        mpc_callback=lambda: None,
        home_callback=lambda: None,
        arm_mpc_controller=None,
    ):
        self.mpc_callback = mpc_callback
        self.home_callback = home_callback
        self.arm_mpc_controller = arm_mpc_controller

        # Base EE target position (updated on MPC switch to current EE pos)
        self.base_x = 0.4
        self.base_y = 0.0
        self.base_z = 0.3

        # Joystick offset scale (meters per full deflection)
        self.xy_scale = 0.3
        self.z_scale = 0.3

        # EE orientation: pointing down (180 deg around Y), xyzw format
        self.orientation = np.array([0.0, 1.0, 0.0, 0.0])

        # Robot interface for reading joint state on MPC switch
        self.robot = None

        self.root = tk.Tk()
        self.root.title("Franka EE Joystick")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack()

        joystick_frame = ttk.Frame(main_frame)
        joystick_frame.pack()

        # Left joystick: X/Y position
        left_frame = ttk.Frame(joystick_frame)
        left_frame.grid(row=0, column=0, padx=15)
        left_title = ttk.Label(left_frame, text="EE X / Y", font=("Arial", 12, "bold"))
        left_title.pack(pady=(0, 5))
        self.left_joystick = VirtualJoystick(left_frame, size=160)
        self.left_joystick.pack()

        # Right joystick: Z position (only vertical axis used)
        right_frame = ttk.Frame(joystick_frame)
        right_frame.grid(row=0, column=1, padx=15)
        right_title = ttk.Label(right_frame, text="EE Z (up/down)", font=("Arial", 12, "bold"))
        right_title.pack(pady=(0, 5))
        self.right_joystick = VirtualJoystick(right_frame, size=160)
        self.right_joystick.pack()

        # Buttons
        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.pack(pady=10)

        self.btn_home = ttk.Button(buttons_frame, text="Home", command=self.home_callback)
        self.btn_home.grid(row=0, column=0, padx=10)

        self.btn_mpc = ttk.Button(buttons_frame, text="MPC", command=self.mpc_callback)
        self.btn_mpc.grid(row=0, column=1, padx=10)

        # Status label
        self.status_var = tk.StringVar(value="EE target: (0.50, 0.00, 0.50)")
        self.status_label = ttk.Label(main_frame, textvariable=self.status_var, font=("Courier", 10))
        self.status_label.pack(pady=(5, 0))

        self.root.update_idletasks()
        self.root.after(50, self.update_loop)

    def update_loop(self):
        # Compute EE target from joystick offsets
        target_x = self.base_x + self.left_joystick.current_y * self.xy_scale
        target_y = self.base_y - self.left_joystick.current_x * self.xy_scale
        target_z = self.base_z + self.right_joystick.current_y * self.z_scale

        self.status_var.set(f"EE target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")

        # Send to controller
        if self.arm_mpc_controller is not None:
            position = np.array([target_x, target_y, target_z])
            self.arm_mpc_controller.set_target_ee_pose(position, self.orientation)

        self.root.after(50, self.update_loop)

    def snapshot_current_ee_orientation(self):
        """Read current EE orientation via pinocchio FK and use it as the target orientation."""
        if self.robot is None:
            return
        import pinocchio
        from scipy.spatial.transform import Rotation

        state = self.robot.get_latest_state()
        q = np.array(state.x[:7])

        model = pinocchio.buildModelFromUrdf(
            str(tbai_descriptions.get_urdf_path("franka_panda")),
        )
        data = model.createData()
        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacements(model, data)
        ee_frame_id = model.getFrameId("panda_hand_tcp")
        R = data.oMf[ee_frame_id].rotation
        quat = Rotation.from_matrix(R).as_quat()  # xyzw
        self.orientation = quat
        print(f"Snapshotted EE orientation (xyzw): {quat}")
        self.update_loop()

    def on_closing(self):
        self.root.quit()

    def run(self):
        self.root.mainloop()


CONFIG_FILE = os.path.join(os.path.dirname(__file__), "franka_config.yaml")
TASK_FILE = os.path.join(os.path.dirname(__file__), "task.info")

os.environ["TBAI_ROBOT_DESCRIPTION_PATH"] = str(tbai_descriptions.get_urdf_path("franka_panda"))
os.environ["TBAI_GLOBAL_CONFIG_PATH"] = str(CONFIG_FILE)
os.environ["TBAI_TASK_FILE_PATH"] = str(TASK_FILE)


@dataclasses.dataclass
class Args:
    camera: bool = False
    camera_topic: str = "rt/camera/image"


class FrankaChangeControllerSubscriber(ChangeControllerSubscriber):
    def __init__(self):
        super().__init__()
        self._callback = None
        self.new_controller = None

    def set_callback_function(self, callback):
        self._callback = callback

    def trigger_callbacks(self):
        if self._callback is not None and self.new_controller is not None:
            self._callback(str(self.new_controller))
            self.new_controller = None

    def home_callback(self):
        self.new_controller = "STAND"

    def mpc_callback(self):
        if self._on_mpc_switch is not None:
            self._on_mpc_switch()
        self.new_controller = "MPC"

    _on_mpc_switch = None


class ImageViewer:
    def __init__(self, topic: str):
        from tbai_sdk.subscriber import PollingSubscriber
        from tbai_sdk.messages.robot_msgs import ImgFrame
        import matplotlib.pyplot as plt

        self._sub = PollingSubscriber(ImgFrame, topic)
        self._plt = plt
        self._fig, self._ax = plt.subplots()
        self._img_handle = None
        self._ax.set_axis_off()
        self._fig.tight_layout()
        print(f"ImageViewer: subscribed to {topic}")

    def update(self):
        frame = self._sub.take()
        if frame is None:
            return
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

    if not tbai_python.HAS_DEPLOY_FRANKA:
        print("Error: tbai_python was built without Franka support (TBAI_BUILD_DEPLOY_FRANKA=OFF)")
        sys.exit(1)

    if not tbai_python.HAS_ARM_MPC:
        print("Error: tbai_python was built without ARM MPC support (TBAI_BUILD_MPC=OFF)")
        sys.exit(1)

    robot_args = tbai_python.FrankaRobotInterfaceArgs()
    print("Connecting to Franka...")
    robot = tbai_python.FrankaRobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.wait_till_initialized()
    print("Robot initialized.")

    controller_sub = FrankaChangeControllerSubscriber()

    tbai_python.write_init_time()

    central_controller = tbai_python.CentralController(robot, controller_sub)

    # Static controller (home position)
    static_ctrl = tbai_python.StaticController(robot)

    # Arm MPC controller (uses system clock internally)
    arm_mpc_ctrl = tbai_python.ArmMpcController(robot)

    central_controller.add_controller(static_ctrl)
    central_controller.add_controller(arm_mpc_ctrl)

    # Create joystick UI
    ui = EEJoystickUI(
        mpc_callback=controller_sub.mpc_callback,
        home_callback=controller_sub.home_callback,
        arm_mpc_controller=arm_mpc_ctrl,
    )

    # Snapshot current EE orientation when switching to MPC
    ui.robot = robot
    controller_sub._on_mpc_switch = ui.snapshot_current_ee_orientation

    # Camera viewer
    image_viewer = None
    if args.camera:
        image_viewer = ImageViewer(args.camera_topic)
        image_viewer.start_nonblocking()

    central_controller.start_thread()

    try:
        if image_viewer:

            def poll_image():
                image_viewer.update()
                ui.root.after(33, poll_image)

            ui.root.after(33, poll_image)
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if image_viewer:
            image_viewer.close()
        print("Stopping controller...")
        central_controller.stop_thread()
        print("Done.")


if __name__ == "__main__":
    main()
