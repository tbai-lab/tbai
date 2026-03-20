import math
import tkinter as tk
from tkinter import ttk


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
            ttk.Button(
                row1_frame, text=label, width=10,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=5)

        # Row 2: Mimic dance controllers
        row2_frame = ttk.Frame(button_grid)
        row2_frame.pack(pady=(10, 0))

        for name, label in [("G1MimicDance102", "DANCE"), ("G1MimicGangnam", "GANGNAM")]:
            ttk.Button(
                row2_frame, text=label, width=10,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=5)

        # Row 3: BeyondMimic controllers
        row3_frame = ttk.Frame(button_grid)
        row3_frame.pack(pady=(10, 0))
        ttk.Label(row3_frame, text="beyond mimic:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1BeyondDance", "B-DANCE"), ("G1Spinkick", "SPINKICK")]:
            ttk.Button(
                row3_frame, text=label, width=8,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

        # TWIST2 controllers
        twist_frame = ttk.Frame(button_grid)
        twist_frame.pack(pady=(10, 0))
        ttk.Label(twist_frame, text="twist2:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1TwistWalk1", "WALK1"), ("G1TwistWalk2", "WALK2"),
                            ("G1TwistWalk3", "WALK3"), ("G1TwistWalk5", "WALK5"),
                            ("G1TwistWalk7", "WALK7"), ("G1TwistSwing", "SWING")]:
            ttk.Button(
                twist_frame, text=label, width=7,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

        # PBHC controllers
        pbhc_frame = ttk.Frame(button_grid)
        pbhc_frame.pack(pady=(10, 0))
        ttk.Label(pbhc_frame, text="pbhc:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1PBHCHorseStancePunch", "PUNCH"),
                            ("G1PBHCHorseStancePose", "POSE"),
                            ("G1PBHCHorseStancePose2", "POSE2")]:
            ttk.Button(
                pbhc_frame, text=label, width=7,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

        # ASAP controllers - row 1
        asap_frame1 = ttk.Frame(button_grid)
        asap_frame1.pack(pady=(10, 0))
        ttk.Label(asap_frame1, text="asap:", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1ASAPLocomotion", "LOCO"), ("G1ASAPCR7", "CR7"),
                            ("G1ASAPAPT", "APT"), ("G1ASAPKobe", "KOBE")]:
            ttk.Button(
                asap_frame1, text=label, width=7,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

        # ASAP controllers - row 2
        asap_frame2 = ttk.Frame(button_grid)
        asap_frame2.pack(pady=(5, 0))
        ttk.Label(asap_frame2, text="", width=6).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1ASAPJumpForward1", "JUMP1"), ("G1ASAPJumpForward2", "JUMP2"),
                            ("G1ASAPJumpForward3", "JUMP3"), ("G1ASAPSideJump1", "SIDE1"),
                            ("G1ASAPSideJump2", "SIDE2"), ("G1ASAPSideJump3", "SIDE3")]:
            ttk.Button(
                asap_frame2, text=label, width=7,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

        # ASAP controllers - row 3
        asap_frame3 = ttk.Frame(button_grid)
        asap_frame3.pack(pady=(5, 0))
        ttk.Label(asap_frame3, text="", width=6).pack(side=tk.LEFT, padx=5)

        for name, label in [("G1ASAPKick1", "KICK1"), ("G1ASAPKick2", "KICK2"),
                            ("G1ASAPKick3", "KICK3"), ("G1ASAPLeBron1", "LEBRON1"),
                            ("G1ASAPLeBron2", "LEBRON2")]:
            ttk.Button(
                asap_frame3, text=label, width=7,
                command=lambda n=name: self._on_controller(n)
            ).pack(side=tk.LEFT, padx=3)

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


def main():
    try:
        ui_controller = G1UIController()
        ui_controller.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
