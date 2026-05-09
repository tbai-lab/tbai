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
    unitree: bool = False
    net: str = "lo"
    channel: int = 1
    no_lidar: bool = False
    log: bool = False
    video: bool = False
    pointcloud: bool = False
    pointcloud_topic: str = "rt/pointcloud"
    # Visual odometry: feed SuperPoint-derived landmarks into the InEKF.
    enable_vo: bool = False
    superpoint_onnx: str = ""  # path to superpoint.onnx (download via tbai_estim/scripts/download_superpoint_onnx.py)
    image_topic: str = "rt/camera/image"
    depth_topic: str = "rt/camera/depth"  # ImgFrame, encoding 32FC1, published by tbai_mujoco's DepthImagePublisher
    vo_max_keypoints: int = 64
    vo_input_width: int = 320
    vo_input_height: int = 240
    vo_rate_hz: float = 3.0
    vo_intra_op_threads: int = 2  # ORT intra-op parallelism for SuperPoint
    show_vo: bool = False  # display the keypoint overlay in a cv2 window
    # Pinhole intrinsics for the depth camera, used to back-project (u, v, depth)
    # to (X, Y, Z) in the camera frame. Defaults match a 640x480 MuJoCo camera
    # with the default 45 deg vertical FOV (fy = 240 / tan(22.5 deg) ~= 579.4).
    depth_fx: float = 579.4
    depth_fy: float = 579.4
    depth_cx: float = 320.0
    depth_cy: float = 240.0


class Go2ChangeControllerSubscriber(ChangeControllerSubscriber):
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

    def get_reference_velocity(self, time, dt):
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

        current_state = self.state_subscriber.get_latest_state()
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


class VisualOdometryWorker:
    """Background thread that pumps (rgb image, depth image) -> SuperPoint -> tracker -> robot.

    Subscribes to two ImgFrame topics via tbai_sdk: an RGB-style image and a
    float32 depth image (encoding "32FC1") published by tbai_mujoco's
    DepthImagePublisher. On each tick takes the most recent of each, back-
    projects the depth into a per-pixel (X, Y, Z) buffer, runs SuperPoint +
    tracker, and forwards landmarks to the robot's internal InEKF via
    correct_visual_landmarks.
    """

    def __init__(self, robot, onnx_path, image_topic, depth_topic, *, max_keypoints, input_width, input_height,
                 rate_hz, fx, fy, cx, cy, intra_op_threads=2, show=False):
        import threading
        import numpy as np
        from tbai_sdk.subscriber import PollingSubscriber
        from tbai_sdk.messages.robot_msgs import ImgFrame

        if not tbai_python.HAS_VO:
            raise RuntimeError("tbai_python was built without TBAI_BUILD_VO=ON; visual odometry unavailable.")
        if not hasattr(robot, "correct_visual_landmarks"):
            raise RuntimeError(
                "Robot interface has no correct_visual_landmarks(...) method; rebuild tbai_python with VO enabled."
            )

        self._robot = robot
        self._image_topic = image_topic
        self._depth_topic = depth_topic
        self._image_sub = PollingSubscriber(ImgFrame, image_topic)
        self._depth_sub = PollingSubscriber(ImgFrame, depth_topic)
        print(f"VO: subscribed to image={image_topic}  depth={depth_topic}")

        opts = tbai_python.vision.SuperPointOptions()
        opts.input_width = input_width
        opts.input_height = input_height
        opts.max_keypoints = max_keypoints
        opts.intra_op_threads = intra_op_threads
        self._extractor = tbai_python.vision.SuperPointExtractor(onnx_path, opts)
        self._tracker = tbai_python.vision.VisualLandmarkTracker()
        print(f"VO: SuperPoint loaded from {onnx_path} ({input_width}x{input_height}, k<={max_keypoints})")

        # Pinhole intrinsics for back-projection
        self._fx, self._fy, self._cx, self._cy = float(fx), float(fy), float(cx), float(cy)
        # Pre-build a (H, W) grid of (u-cx)/fx and (v-cy)/fy lazily on first depth frame
        # (we only know the resolution from the first message).
        self._u_norm = None
        self._v_norm = None
        self._cloud_buf = None  # reusable (H, W, 3) float32 buffer

        self._show = show
        self._dt = 1.0 / max(rate_hz, 0.1)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="vo-worker", daemon=True)
        # Counters
        self._image_received = 0
        self._depth_received = 0
        self._processed = 0
        self._fed = 0
        # Skip-reason counters (so "processed=0" tells you WHY)
        self._skip_no_image = 0
        self._skip_no_depth = 0
        self._skip_decode = 0
        self._skip_depth_encoding = 0
        self._skip_dim_mismatch = 0
        self._skip_no_keypoints = 0
        self._skip_no_landmarks = 0
        self._last_img_dims = None
        self._last_depth_dims = None
        self._depth_stats_logged = False
        self._latest_overlay = None  # for visualization, set by worker thread
        # Stage timings (sum of microseconds, count) -- printed alongside the
        # diagnostic line so we can see where the budget is going.
        self._t_decode = (0, 0)
        self._t_backproj = (0, 0)
        self._t_detect = (0, 0)
        self._t_track = (0, 0)
        self._t_correct = (0, 0)

    def start(self):
        self._thread.start()
        if self._show:
            print("VO: --show-vo enabled (cv2 window updates from main thread on poll)")

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._show:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
        self._print_diagnostics(prefix="VO: stopped.")

    # Called from main thread to push a cv2.imshow update (cv2 GUI must be on main thread on macOS;
    # on Linux it's lenient, but we keep the worker free of GUI calls just in case).
    def pump_visualization(self):
        if not self._show:
            return
        overlay = self._latest_overlay
        if overlay is None:
            return
        try:
            import cv2
            cv2.imshow("VO: SuperPoint keypoints (green=tracked, red=new)", overlay)
            cv2.waitKey(1)
        except Exception as e:
            print(f"VO: cv2 imshow failed: {e}")
            self._show = False

    def _decode_image(self, frame):
        import numpy as np
        enc = frame.encoding
        if enc in ("bgr8", "rgb8"):
            arr = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 3)
            # SuperPointExtractor expects BGR for 3-channel; flip if it's RGB.
            return arr if enc == "bgr8" else arr[:, :, ::-1].copy()
        if enc == "mono8":
            return np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width)
        if enc == "rgba8":
            arr = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 4)
            return arr[:, :, [2, 1, 0]].copy()  # to BGR
        return None

    def _backproject_depth(self, depth_frame):
        """Back-project a 32FC1 depth ImgFrame into a (H, W, 3) float32 (X, Y, Z) buffer.

        Returns (cloud_buf_view, height, width) on success or None on failure.
        Reuses self._cloud_buf to avoid per-frame allocation.
        """
        import numpy as np
        if depth_frame.encoding != "32FC1":
            self._skip_depth_encoding += 1
            return None
        H, W = int(depth_frame.height), int(depth_frame.width)
        depth = np.frombuffer(depth_frame.data, dtype=np.float32).reshape(H, W)

        if self._cloud_buf is None or self._cloud_buf.shape != (H, W, 3):
            us = np.arange(W, dtype=np.float32)
            vs = np.arange(H, dtype=np.float32)
            self._u_norm = ((us - self._cx) / self._fx)[None, :]  # (1, W)
            self._v_norm = ((vs - self._cy) / self._fy)[:, None]  # (H, 1)
            self._cloud_buf = np.empty((H, W, 3), dtype=np.float32)
            print(f"VO: depth back-projection grid built: {W}x{H} fx={self._fx:.1f} fy={self._fy:.1f} "
                  f"cx={self._cx:.1f} cy={self._cy:.1f}")

        # X = (u - cx) * Z / fx; Y = (v - cy) * Z / fy; Z stays. NaNs propagate.
        np.multiply(depth, self._u_norm, out=self._cloud_buf[:, :, 0])
        np.multiply(depth, self._v_norm, out=self._cloud_buf[:, :, 1])
        self._cloud_buf[:, :, 2] = depth

        if not self._depth_stats_logged:
            valid = np.isfinite(depth)
            n_valid = int(valid.sum())
            if n_valid > 0:
                z = depth[valid]
                print(f"VO: first depth frame: shape={depth.shape} valid={n_valid}/{H*W} "
                      f"z=[{float(z.min()):.3f}, {float(z.max()):.3f}] median={float(np.median(z)):.3f}")
                self._depth_stats_logged = True

        return self._cloud_buf, H, W

    def _print_diagnostics(self, prefix="VO:"):
        def avg_us(stage):
            total, n = stage
            return f"{total // n}us" if n > 0 else "-"

        msg = (
            f"{prefix} img_rx={self._image_received} depth_rx={self._depth_received} "
            f"processed={self._processed} fed={self._fed} | "
            f"skips: no_img={self._skip_no_image} no_depth={self._skip_no_depth} "
            f"decode={self._skip_decode} depth_enc={self._skip_depth_encoding} "
            f"dim_mismatch={self._skip_dim_mismatch} "
            f"no_kps={self._skip_no_keypoints} no_lms={self._skip_no_landmarks} | "
            f"avg(us): decode={avg_us(self._t_decode)} backproj={avg_us(self._t_backproj)} "
            f"detect={avg_us(self._t_detect)} track={avg_us(self._t_track)} "
            f"correct={avg_us(self._t_correct)}"
        )
        if self._last_img_dims is not None or self._last_depth_dims is not None:
            msg += f" | last img={self._last_img_dims} depth={self._last_depth_dims}"
        print(msg)

    def _build_overlay(self, image, kps, ids_for_kps, prev_ids):
        import numpy as np
        import cv2
        bgr = image if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        bgr = bgr.copy()
        prev_set = set(prev_ids) if prev_ids else set()
        for (x, y), kpid in zip(kps, ids_for_kps):
            color = (0, 220, 0) if kpid in prev_set else (0, 0, 255)  # green=tracked, red=new
            cv2.circle(bgr, (int(round(x)), int(round(y))), 3, color, 1, lineType=cv2.LINE_AA)
        cv2.putText(bgr, f"kps={len(kps)} tracked={sum(1 for i in ids_for_kps if i in prev_set)}",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        return bgr

    def _run(self):
        import time
        import numpy as np
        last_diag = time.monotonic()
        prev_landmark_ids = []
        while not self._stop_event.is_set():
            start = time.monotonic()
            frame = self._image_sub.take()
            depth = self._depth_sub.take()
            if frame is not None:
                self._image_received += 1
                self._last_img_dims = (frame.width, frame.height, frame.encoding)
            else:
                self._skip_no_image += 1
            if depth is not None:
                self._depth_received += 1
                self._last_depth_dims = (depth.width, depth.height, depth.encoding)
            else:
                self._skip_no_depth += 1

            if frame is not None and depth is not None:
                t = time.monotonic()
                image = self._decode_image(frame)
                self._t_decode = (self._t_decode[0] + int((time.monotonic() - t) * 1e6), self._t_decode[1] + 1)
                if image is None:
                    self._skip_decode += 1
                else:
                    t = time.monotonic()
                    bp = self._backproject_depth(depth)
                    self._t_backproj = (self._t_backproj[0] + int((time.monotonic() - t) * 1e6), self._t_backproj[1] + 1)
                    if bp is not None:
                        cloud_buf, dh, dw = bp
                        if dh != frame.height or dw != frame.width:
                            self._skip_dim_mismatch += 1
                        else:
                            try:
                                cloud_bytes = cloud_buf.view(np.uint8).reshape(-1)
                                indexer = tbai_python.vision.PointCloudIndexer(
                                    cloud_bytes, dh, dw, dw * 3 * 4, 3 * 4, 0, 8.0,
                                )
                                t = time.monotonic()
                                kps, scores, desc = self._extractor.detect(image)
                                self._t_detect = (self._t_detect[0] + int((time.monotonic() - t) * 1e6),
                                                  self._t_detect[1] + 1)
                                if kps.shape[0] == 0:
                                    self._skip_no_keypoints += 1
                                else:
                                    t = time.monotonic()
                                    landmarks = self._tracker.process_frame(kps, desc, indexer)
                                    self._t_track = (self._t_track[0] + int((time.monotonic() - t) * 1e6),
                                                     self._t_track[1] + 1)
                                    if landmarks:
                                        ids = [lm.id for lm in landmarks]
                                        t = time.monotonic()
                                        self._robot.correct_visual_landmarks(landmarks)
                                        self._t_correct = (self._t_correct[0] + int((time.monotonic() - t) * 1e6),
                                                           self._t_correct[1] + 1)
                                        self._fed += 1
                                        if self._show:
                                            kp_ids = ids if len(ids) == kps.shape[0] else list(range(kps.shape[0]))
                                            self._latest_overlay = self._build_overlay(
                                                image, kps, kp_ids, prev_landmark_ids
                                            )
                                        prev_landmark_ids = ids
                                    else:
                                        self._skip_no_landmarks += 1
                                self._processed += 1
                            except Exception as e:
                                print(f"VO: pipeline error: {e}")

            now = time.monotonic()
            if now - last_diag >= 2.0:
                self._print_diagnostics()
                last_diag = now

            elapsed = now - start
            sleep = self._dt - elapsed
            if sleep > 0:
                self._stop_event.wait(sleep)


def main():
    args = tyro.cli(Args)

    if args.unitree:
        if not tbai_python.HAS_DEPLOY_GO2_UNITREE:
            print("Error: tbai_python was built without Go2 Unitree support (TBAI_BUILD_DEPLOY_GO2_UNITREE=OFF)")
            sys.exit(1)

        robot_args = tbai_python.Go2RobotInterfaceUnitreeArgs()
        robot_args.network_interface = args.net
        robot_args.unitree_channel = args.channel

        print(f"Connecting to Go2 (Unitree) on {args.net} (channel {args.channel})...")
        robot = tbai_python.Go2RobotInterfaceUnitree(robot_args)
    else:
        if not tbai_python.HAS_DEPLOY_GO2:
            print("Error: tbai_python was built without Go2 support (TBAI_BUILD_DEPLOY_GO2=OFF)")
            sys.exit(1)

        robot_args = tbai_python.Go2RobotInterfaceArgs()

        print(f"Connecting to Go2 on {args.net} (channel {args.channel})...")
        robot = tbai_python.Go2RobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.wait_till_initialized()
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

    central_controller = tbai_python.CentralController(robot, controller_sub)

    static_ctrl = tbai_python.StaticController(robot, rerun_logger)
    np3o_ctrl = tbai_python.Np3oController(robot, ref_vel_gen, rerun_logger)
    bob_ctrl = tbai_python.BobController(robot, ref_vel_gen, rerun_logger)

    central_controller.add_controller(static_ctrl)
    central_controller.add_controller(np3o_ctrl)
    central_controller.add_controller(bob_ctrl)

    central_controller.start_thread()

    vo_worker = None
    if args.enable_vo:
        if not args.superpoint_onnx:
            print("Error: --enable-vo requires --superpoint-onnx PATH (run "
                  "tbai_estim/scripts/download_superpoint_onnx.py first)")
            central_controller.stop_thread()
            sys.exit(1)
        try:
            vo_worker = VisualOdometryWorker(
                robot,
                onnx_path=args.superpoint_onnx,
                image_topic=args.image_topic,
                depth_topic=args.depth_topic,
                max_keypoints=args.vo_max_keypoints,
                input_width=args.vo_input_width,
                input_height=args.vo_input_height,
                rate_hz=args.vo_rate_hz,
                fx=args.depth_fx,
                fy=args.depth_fy,
                cx=args.depth_cx,
                cy=args.depth_cy,
                intra_op_threads=args.vo_intra_op_threads,
                show=args.show_vo,
            )
            vo_worker.start()
            if args.show_vo:
                # Pump the cv2 window from the Tk main loop (~30 Hz).
                def _pump():
                    vo_worker.pump_visualization()
                    ui_controller.root.after(33, _pump)
                ui_controller.root.after(100, _pump)
        except Exception as e:
            print(f"Failed to start visual odometry worker: {e}")
            central_controller.stop_thread()
            sys.exit(1)

    try:
        ui_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        if vo_worker is not None:
            print("Stopping visual odometry worker...")
            vo_worker.stop()
        print("Stopping controller...")
        central_controller.stop_thread()
        if args.log:
            from tbai_logging.rerun.utils import rerun_store

            rerun_store("go2_deploy_np3o.rrd")
        print("Done.")


if __name__ == "__main__":
    main()
