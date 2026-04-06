#!/usr/bin/env python3

import math
import os
import sys
import tkinter as tk
from tkinter import ttk

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

        self.canvas.coords(
            self.knob,
            x - self.knob_size // 2,
            y - self.knob_size // 2,
            x + self.knob_size // 2,
            y + self.knob_size // 2,
        )

        self.current_x = dx / self.max_distance
        self.current_y = -dy / self.max_distance


class UIController:
    GAITS = ["stance", "trot", "standing_trot", "flying_trot", "pace", "standing_pace",
             "dynamic_walk", "static_walk", "amble"]

    def __init__(self, stand_callback, sit_callback, mpc_callback, set_gait_callback):
        self.root = tk.Tk()
        self.root.title("Go2 MPC Controller")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack()

        joystick_frame = ttk.Frame(main_frame)
        joystick_frame.pack()

        left_frame = ttk.Frame(joystick_frame)
        left_frame.grid(row=0, column=0, padx=15)
        ttk.Label(left_frame, text="X+Y Velocity", font=("Arial", 12, "bold")).pack(pady=(0, 5))
        self.left_joystick = VirtualJoystick(left_frame, size=160)
        self.left_joystick.pack()

        right_frame = ttk.Frame(joystick_frame)
        right_frame.grid(row=0, column=1, padx=15)
        ttk.Label(right_frame, text="X+Yaw Velocity", font=("Arial", 12, "bold")).pack(pady=(0, 5))
        self.right_joystick = VirtualJoystick(right_frame, size=160)
        self.right_joystick.pack()

        # Controller buttons
        ctrl_frame = ttk.Frame(main_frame)
        ctrl_frame.pack(pady=(10, 5))
        ttk.Button(ctrl_frame, text="Stand", command=stand_callback).grid(row=0, column=0, padx=5)
        ttk.Button(ctrl_frame, text="Sit", command=sit_callback).grid(row=0, column=1, padx=5)
        ttk.Button(ctrl_frame, text="MPC", command=mpc_callback).grid(row=0, column=2, padx=5)

        # Gait buttons
        gait_frame = ttk.LabelFrame(main_frame, text="Gaits", padding="5")
        gait_frame.pack(pady=5, fill="x")
        for i, gait in enumerate(self.GAITS):
            ttk.Button(gait_frame, text=gait.replace("_", " "),
                       command=lambda g=gait: set_gait_callback(g)).grid(row=i // 5, column=i % 5, padx=3, pady=2)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.linear_x_scale = 1.0
        self.linear_y_scale = 0.5
        self.angular_scale = 0.5

        self.root.update_idletasks()
        self.root.after(100, self._update_gui)

    def _update_gui(self):
        # Left joystick: X+Y velocity
        self.linear_x = self.left_joystick.current_y * self.linear_x_scale
        self.linear_y = -self.left_joystick.current_x * self.linear_y_scale

        # Right joystick: X+Yaw velocity (X overrides left if moved)
        right_x_vel = self.right_joystick.current_y * self.linear_x_scale
        self.angular_z = -self.right_joystick.current_x * self.angular_scale
        if abs(right_x_vel) > 0.01:
            self.linear_x = right_x_vel

        self.root.after(50, self._update_gui)

    def on_closing(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.root.quit()

    def run(self):
        self.root.mainloop()


class MpcChangeControllerSubscriber(ChangeControllerSubscriber):
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

    def stand_callback(self):
        self.new_controller = "STAND"

    def sit_callback(self):
        self.new_controller = "SIT"

    def mpc_callback(self):
        self.new_controller = "WBC"


class MpcReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: UIController, ramped_velocity: float = 5.0):
        super().__init__()
        self.ui_controller = ui_controller
        self.ramped_velocity = ramped_velocity
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def get_reference_velocity(self, time, dt):
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


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(SCRIPT_DIR, "config")
CONFIG_FILE = os.path.join(SCRIPT_DIR, "go2_mpc_config.yaml")

os.environ["TBAI_ROBOT_DESCRIPTION_PATH"] = str(tbai_descriptions.get_urdf_path("go2"))
os.environ["TBAI_GLOBAL_CONFIG_PATH"] = str(CONFIG_FILE)


def main():
    if not tbai_python.HAS_QUADRUPED_MPC:
        print("Error: tbai was built without quadruped MPC support (TBAI_BUILD_MPC=OFF)")
        sys.exit(1)

    if not tbai_python.HAS_DEPLOY_GO2:
        print("Error: tbai was built without Go2 support (TBAI_BUILD_DEPLOY_GO2=OFF)")
        sys.exit(1)

    robot_args = tbai_python.Go2RobotInterfaceArgs()
    print("Connecting to Go2...")
    robot = tbai_python.Go2RobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.wait_till_initialized()
    print("Robot initialized.")

    controller_sub = MpcChangeControllerSubscriber()

    mpc_ctrl_holder = [None]

    def set_gait_callback(gait_name):
        if mpc_ctrl_holder[0] is not None:
            mpc_ctrl_holder[0].set_gait(gait_name)

    ui = UIController(
        stand_callback=controller_sub.stand_callback,
        sit_callback=controller_sub.sit_callback,
        mpc_callback=controller_sub.mpc_callback,
        set_gait_callback=set_gait_callback,
    )
    ref_vel_gen = MpcReferenceVelocityGenerator(ui)

    tbai_python.write_init_time()

    mpc_ctrl = tbai_python.QuadrupedMpcController(
        robot_name="go2",
        robot_interface=robot,
        velocity_generator=ref_vel_gen,
        urdf_path=str(tbai_descriptions.get_urdf_path("go2")),
        task_settings_file=os.path.join(CONFIG_DIR, "task.info"),
        frame_declaration_file=os.path.join(CONFIG_DIR, "frame_declarations.info"),
        controller_config_file=os.path.join(CONFIG_DIR, "controllers.info"),
        target_command_file=os.path.join(CONFIG_DIR, "targetCommand.info"),
        sqp_settings_file=os.path.join(CONFIG_DIR, "sqp.info"),
        gait_file=os.path.join(CONFIG_DIR, "gait.info"),
    )
    mpc_ctrl_holder[0] = mpc_ctrl

    central_controller = tbai_python.CentralController(robot, controller_sub)
    static_ctrl = tbai_python.StaticController(robot)

    central_controller.add_controller(static_ctrl)
    central_controller.add_controller(mpc_ctrl)

    central_controller.start_thread()

    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping controller...")
        central_controller.stop_thread()
        print("Done.")


if __name__ == "__main__":
    main()
