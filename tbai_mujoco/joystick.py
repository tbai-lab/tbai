import math
import io
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
        """Set callback function to be called when joystick position changes"""
        self.command_callback = callback

    def on_click(self, event):
        self.move_knob(event.x, event.y)

    def on_drag(self, event):
        self.move_knob(event.x, event.y)

    def on_release(self, event):
        # Return to center
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
    def __init__(self, stand_callback = lambda: None, sit_callback = lambda: None, np3o_callback = lambda: None, bob_callback = lambda: None, robot=None, show_video=False, show_pointcloud=False):
        self.stand_callback = stand_callback
        self.sit_callback = sit_callback
        self.bob_callback = bob_callback
        self.np3o_callback = np3o_callback
        self.robot = robot
        self.show_video = show_video
        self.show_pointcloud = show_pointcloud

        self.root = tk.Tk()
        self.root.title("Virtual Joystick")
        self.root.resizable(False, False)  # Prevent resizing
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

        # Add buttons frame
        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.pack(pady=10)

        # Create and pack the buttons
        self.button1 = ttk.Button(buttons_frame, text="Stand", command=self.stand_callback)
        self.button1.grid(row=0, column=0, padx=10)

        self.button2 = ttk.Button(buttons_frame, text="Sit", command=self.sit_callback)
        self.button2.grid(row=0, column=1, padx=10)

        self.button3 = ttk.Button(buttons_frame, text="np3o", command=self.np3o_callback)
        self.button3.grid(row=0, column=2, padx=10)

        self.button4 = ttk.Button(buttons_frame, text="bob", command=self.bob_callback)
        self.button4.grid(row=0, column=3, padx=10)

        # Camera feeds frame (side by side)
        if self.robot is not None and (self.show_video or self.show_pointcloud):
            feeds_frame = ttk.Frame(main_frame)
            feeds_frame.pack(pady=10)

            self._cam_label = ttk.Label(feeds_frame)
            self._cam_label.grid(row=0, column=0, padx=5)
            self._cam_photo = None

            if self.show_pointcloud:
                self._pc3d_label = ttk.Label(feeds_frame)
                self._pc3d_label.grid(row=0, column=1, padx=5)
                self._pc3d_photo = None
                self._pc3d_fig = plt.figure(figsize=(4, 3), dpi=100)
                self._pc3d_ax = self._pc3d_fig.add_subplot(111, projection="3d")
                self._pc3d_angle = 0

            self.root.after(100, self._update_camera)
            if self.show_pointcloud:
                self.root.after(300, self._update_3d_pointcloud)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.linear_x_scale = 1.0  # m/s
        self.linear_y_scale = 1.0  # m/s
        self.angular_scale = 1.0  # rad/s

        self.root.update_idletasks()
        self.root.after(100, self.update_gui)

    def update_xy_velocity(self, x, y, z):
        """Handle X+Y velocity from left joystick"""
        self.linear_x = y * self.linear_x_scale
        self.linear_y = -x * self.linear_y_scale

    def update_x_yaw_velocity(self, x, y, z):
        """Handle X+Yaw velocity from right joystick"""
        self.linear_x = y * self.linear_x_scale
        self.angular_z = -x * self.angular_scale

    def get_state(self):
        """Return current joystick state"""
        return self.linear_x, self.linear_y, self.angular_z

    def _depth_to_colormap(self, depth_img):
        """Convert a 2D depth array to an RGB heatmap. Close=hot, far=cold."""
        valid = depth_img > 0
        if not np.any(valid):
            return np.zeros((*depth_img.shape, 3), dtype=np.uint8)
        d_min = np.min(depth_img[valid])
        d_max = np.max(depth_img[valid])
        if d_max - d_min < 1e-6:
            d_max = d_min + 1.0
        # Invert so close=1 (hot), far=0 (cold)
        norm = 1.0 - np.clip((depth_img - d_min) / (d_max - d_min), 0, 1)
        # Inferno-like: black -> red -> yellow -> white
        r = np.clip(norm * 3.0, 0, 1)
        g = np.clip(norm * 3.0 - 1.0, 0, 1)
        b = np.clip(norm * 3.0 - 2.0, 0, 1)
        rgb = np.stack([r, g, b], axis=-1)
        rgb[~valid] = 0
        return (rgb * 255).astype(np.uint8)

    def _update_camera(self):
        """Fetch RGB + depth, overlay, and display."""
        try:
            rgb = None
            if self.show_video:
                img = self.robot.getLatestImage()
                if img is not None:
                    rgb = img[:, :, ::-1].copy()  # BGR -> RGB

            if self.show_pointcloud and rgb is not None:
                pts = self.robot.getLatestPointcloud()
                if pts is not None and len(pts) > 0:
                    h, w = rgb.shape[:2]
                    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

                    # Project back to image coords (camera frame: z forward, x right, y down)
                    # fovy=70 deg, square pixels
                    fovy = 70.0 * np.pi / 180.0
                    fy = h / (2.0 * np.tan(fovy / 2.0))
                    fx = fy
                    cx, cy = w / 2.0, h / 2.0

                    mask = (z > 0) & (z < 3.0)
                    u = (fx * x[mask] / z[mask] + cx).astype(np.int32)
                    v = (fy * y[mask] / z[mask] + cy).astype(np.int32)
                    d = z[mask]

                    in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                    u, v, d = u[in_bounds], v[in_bounds], d[in_bounds]

                    # Fill a small square around each point to cover stride gaps
                    depth_img = np.zeros((h, w), dtype=np.float32)
                    radius = 2
                    for du in range(-radius, radius + 1):
                        for dv in range(-radius, radius + 1):
                            uc = np.clip(u + du, 0, w - 1)
                            vc = np.clip(v + dv, 0, h - 1)
                            depth_img[vc, uc] = d

                    heatmap = self._depth_to_colormap(depth_img)
                    alpha = 0.4
                    overlay_mask = depth_img > 0
                    rgb[overlay_mask] = (
                        (1 - alpha) * rgb[overlay_mask] + alpha * heatmap[overlay_mask]
                    ).astype(np.uint8)

            if rgb is not None:
                pil_img = Image.fromarray(rgb)
                self._cam_photo = ImageTk.PhotoImage(pil_img)
                self._cam_label.configure(image=self._cam_photo)
        except Exception:
            pass
        self.root.after(100, self._update_camera)

    def _update_3d_pointcloud(self):
        """Render a rotating 3D scatter of the pointcloud."""
        try:
            pts = self.robot.getLatestPointcloud()
            if pts is not None and len(pts) > 0:
                # Filter far points and subsample for speed
                pts = pts[pts[:, 2] < 3.0]
                if len(pts) == 0:
                    self.root.after(300, self._update_3d_pointcloud)
                    return
                step = max(1, len(pts) // 2000)
                pts_s = pts[::step]
                x, y, z = pts_s[:, 0], pts_s[:, 1], pts_s[:, 2]

                # Normalize depth for color
                d_min, d_max = z.min(), z.max()
                if d_max - d_min < 1e-6:
                    d_max = d_min + 1.0
                colors = 1.0 - (z - d_min) / (d_max - d_min)

                ax = self._pc3d_ax
                ax.clear()
                # Camera frame: x=right, y=down, z=forward
                # Plot as: forward=Y axis, right=X axis, up=Z axis
                ax.scatter(x, z, -y, c=colors, cmap="inferno", s=1, depthshade=True)
                ax.set_xlabel("Right")
                ax.set_ylabel("Forward")
                ax.set_zlabel("Up")

                # Equal axis limits
                max_range = 3.0
                mid_x = (x.min() + x.max()) / 2
                mid_y = (z.min() + z.max()) / 2
                mid_z = (-y.min() + -y.max()) / 2
                ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
                ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
                ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

                self._pc3d_angle = (self._pc3d_angle + 2) % 360
                ax.view_init(elev=25, azim=self._pc3d_angle)
                self._pc3d_fig.tight_layout()

                buf = io.BytesIO()
                self._pc3d_fig.savefig(buf, format="png", bbox_inches="tight")
                buf.seek(0)
                pil_img = Image.open(buf)
                self._pc3d_photo = ImageTk.PhotoImage(pil_img)
                self._pc3d_label.configure(image=self._pc3d_photo)
        except Exception:
            pass
        self.root.after(300, self._update_3d_pointcloud)

    def update_gui(self):
        """Update GUI periodically"""
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
        ui_controller = UIController()
        ui_controller.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
