#!/usr/bin/env python3

import math
import os
import sys
import dataclasses
import time
import tkinter as tk
from tkinter import ttk

import tyro

import tbai as tbai_python

from tbai import (
    RobotInterface,
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from tbai import rotations

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


class UIController:
    def __init__(
        self,
        stand_callback=lambda: None,
        sit_callback=lambda: None,
        np3o_callback=lambda: None,
        bob_callback=lambda: None,
        robot=None,
    ):
        self.stand_callback = stand_callback
        self.sit_callback = sit_callback
        self.bob_callback = bob_callback
        self.np3o_callback = np3o_callback
        self.robot = robot

        self.root = tk.Tk()
        self.root.title("Virtual Joystick")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack()

        joystick_frame = ttk.Frame(main_frame)
        joystick_frame.pack()

        left_frame = ttk.Frame(joystick_frame)
        left_frame.grid(row=0, column=0, padx=15)

        left_title = ttk.Label(left_frame, text="X+Y Velocity", font=("Arial", 12, "bold"))
        left_title.pack(pady=(0, 5))

        self.left_joystick = VirtualJoystick(left_frame, size=160)
        self.left_joystick.pack()
        self.left_joystick.set_command_callback(self.update_xy_velocity)

        right_frame = ttk.Frame(joystick_frame)
        right_frame.grid(row=0, column=1, padx=15)

        right_title = ttk.Label(right_frame, text="X+Yaw Velocity", font=("Arial", 12, "bold"))
        right_title.pack(pady=(0, 5))

        self.right_joystick = VirtualJoystick(right_frame, size=160)
        self.right_joystick.pack()
        self.right_joystick.set_command_callback(self.update_x_yaw_velocity)

        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.pack(pady=10)

        self.button1 = ttk.Button(buttons_frame, text="Stand", command=self.stand_callback)
        self.button1.grid(row=0, column=0, padx=10)

        self.button2 = ttk.Button(buttons_frame, text="Sit", command=self.sit_callback)
        self.button2.grid(row=0, column=1, padx=10)

        self.button3 = ttk.Button(buttons_frame, text="np3o", command=self.np3o_callback)
        self.button3.grid(row=0, column=2, padx=10)

        self.button4 = ttk.Button(buttons_frame, text="bob", command=self.bob_callback)
        self.button4.grid(row=0, column=3, padx=10)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.linear_x_scale = 1.0
        self.linear_y_scale = 1.0
        self.angular_scale = 1.0

        self.root.update_idletasks()
        self.root.after(100, self.update_gui)

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


CONFIG_FILE = os.path.join(os.path.dirname(__file__), "go2_config.yaml")

os.environ["TBAI_ROBOT_DESCRIPTION_PATH"] = str(tbai_descriptions.get_urdf_path("go2"))
os.environ["TBAI_GLOBAL_CONFIG_PATH"] = str(CONFIG_FILE)


@dataclasses.dataclass
class Args:
    net: str = "lo"
    channel: int = 1
    no_lidar: bool = False
    log: bool = False
    video: bool = False
    pointcloud: bool = False
    pointcloud_topic: str = "rt/pointcloud"


class Go2ChangeControllerSubscriber(ChangeControllerSubscriber):
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

    def stand_callback(self):
        self.new_controller = "STAND"

    def sit_callback(self):
        self.new_controller = "SIT"

    def np3o_callback(self):
        self.new_controller = "NP3O"

    def bob_callback(self):
        self.new_controller = "BOB"


class Go2ReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: UIController, ramped_velocity: float = 5.0):
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

        x_diff = desired_x - self.current_x
        y_diff = desired_y - self.current_y
        yaw_diff = desired_yaw - self.current_yaw

        sign = lambda v: 1.0 if v >= 0.0 else -1.0
        self.current_x += sign(x_diff) * min(abs(x_diff), max_step)
        self.current_y += sign(y_diff) * min(abs(y_diff), max_step)
        self.current_yaw += sign(yaw_diff) * min(abs(yaw_diff), max_step)

        ref = ReferenceVelocity()
        ref.velocity_x = self.current_x
        ref.velocity_y = self.current_y
        ref.yaw_rate = self.current_yaw
        return ref


class RerunLoggerNode:
    def __init__(self, state_subscriber: RobotInterface, freq=10):
        from tbai_logging.rerun.robot_logger import RobotLogger
        from tbai_logging.rerun.utils import rerun_initialize

        rerun_initialize("go2_deploy_np3o", spawn=False)
        self.robot_logger = RobotLogger.from_zoo("go2_description")
        self.state_subscriber = state_subscriber
        self.freq = freq
        self.last_time = None

    def __call__(self, current_time, dt):
        self.visualize_callback(current_time, dt)

    def visualize_callback(self, current_time, dt):
        if self.last_time is None:
            self.last_time = current_time

        if current_time - self.last_time < 1 / self.freq:
            return
        self.last_time = current_time

        current_state = self.state_subscriber.getLatestState()
        position = current_state.x[3:6]
        orientation = current_state.x[0:3]
        joint_positions = current_state.x[12:24]

        joint_positions = {
            "FL_hip_joint": joint_positions[0],
            "FL_thigh_joint": joint_positions[1],
            "FL_calf_joint": joint_positions[2],
            "RL_hip_joint": joint_positions[3],
            "RL_thigh_joint": joint_positions[4],
            "RL_calf_joint": joint_positions[5],
            "FR_hip_joint": joint_positions[6],
            "FR_thigh_joint": joint_positions[7],
            "FR_calf_joint": joint_positions[8],
            "RR_hip_joint": joint_positions[9],
            "RR_thigh_joint": joint_positions[10],
            "RR_calf_joint": joint_positions[11],
        }

        orientation = rotations.ocs2rpy2quat(orientation)

        self.robot_logger.log_state(
            logtime=current_state.timestamp,
            base_position=position,
            base_orientation=orientation,
            joint_positions=joint_positions,
        )


def main():
    args = tyro.cli(Args)

    if not tbai_python.HAS_DEPLOY_GO2:
        print("Error: tbai_python was built without Go2 support (TBAI_BUILD_DEPLOY_GO2=OFF)")
        sys.exit(1)

    robot_args = tbai_python.Go2RobotInterfaceArgs()
    robot_args.network_interface = args.net
    robot_args.unitree_channel = args.channel
    robot_args.channel_init = True
    robot_args.subscribe_lidar = not args.no_lidar
    robot_args.enable_video = args.video

    print(f"Connecting to Go2 on {args.net} (channel {args.channel})...")
    robot = tbai_python.Go2RobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.waitTillInitialized()
    print("Robot initialized.")

    controller_sub = Go2ChangeControllerSubscriber()

    rerun_logger = RerunLoggerNode(robot) if args.log else None

    ui_controller = UIController(
        stand_callback=controller_sub.stand_callback,
        sit_callback=controller_sub.sit_callback,
        np3o_callback=controller_sub.np3o_callback,
        bob_callback=controller_sub.bob_callback,
        robot=robot if (args.video or args.pointcloud) else None,
    )
    ref_vel_gen = Go2ReferenceVelocityGenerator(ui_controller)

    tbai_python.write_init_time()

    central_controller = tbai_python.CentralController.create(robot, controller_sub)

    static_ctrl = tbai_python.StaticController(robot, rerun_logger)
    np3o_ctrl = tbai_python.Np3oController(robot, ref_vel_gen, rerun_logger)
    bob_ctrl = tbai_python.BobController(robot, ref_vel_gen, rerun_logger)

    central_controller.add_controller(static_ctrl)
    central_controller.add_controller(np3o_ctrl)
    central_controller.add_controller(bob_ctrl)

    central_controller.startThread()

    try:
        ui_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping controller...")
        central_controller.stopThread()
        if args.log:
            from tbai_logging.rerun.utils import rerun_store

            rerun_store("go2_deploy_np3o.rrd")
        print("Done.")


if __name__ == "__main__":
    main()
