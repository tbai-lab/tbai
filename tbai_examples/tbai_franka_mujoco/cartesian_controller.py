from __future__ import annotations

import os
import threading
from typing import Sequence

import numpy as np

import tbai as tbai_python
from scipy.spatial.transform import Rotation
import pinocchio as pin


class CartesianImpedanceController(tbai_python.PythonController):
    JOINT_NAMES: Sequence[str] = tuple(f"panda_joint{i}" for i in range(1, 8))
    DEFAULT_KP = 400.0  # N·m/rad
    DEFAULT_KD = 40.0   # N·m·s/rad
    DEFAULT_RATE_HZ = 50.0
    DEFAULT_EE_FRAME = "panda_hand_tcp"

    # Standard Franka "ready" configuration — the null-space posture the IK
    # regularizer pulls toward, minimizing ||q - q_home||^2 subject to the EE task.
    DEFAULT_Q_HOME: np.ndarray = np.array(
        [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4],
        dtype=np.float64,
    )
    DEFAULT_NULL_GAIN = 0.1

    def __init__(
        self,
        robot_interface,
        urdf_path: str | None = None,
        ee_frame: str = DEFAULT_EE_FRAME,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        rate_hz: float = DEFAULT_RATE_HZ,
        supported_controllers: Sequence[str] = ("CART",),
        q_home: np.ndarray | None = None,
        null_gain: float = DEFAULT_NULL_GAIN,
    ):
        super().__init__(robot_interface, "CartesianImpedance", rate_hz, list(supported_controllers))

        urdf_path = urdf_path or os.environ["TBAI_ROBOT_DESCRIPTION_PATH"]

        self._pin = pin
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._ee_frame_id = self._model.getFrameId(ee_frame)
        self._kp = kp
        self._kd = kd

        # Null-space posture target: biases q toward q_home in the 1D redundancy
        # of the 7-DOF arm + 6-DOF task. Set null_gain=0.0 to disable.
        self._q_home = (
            np.array(q_home, dtype=np.float64) if q_home is not None else self.DEFAULT_Q_HOME.copy()
        )
        self._null_gain = float(null_gain)

        self._lock = threading.Lock()
        self._target_q: np.ndarray | None = None

        # Prebuild MotorCommand templates once to avoid per-tick allocation.
        self._cmds = [
            tbai_python.MotorCommand(name, 0.0, 0.0, self._kp, self._kd, 0.0)
            for name in self.JOINT_NAMES
        ]

    def initialize_hold(self, q_home: np.ndarray) -> None:
        """Seed the held joint target before any EE targets are issued."""
        with self._lock:
            self._target_q = np.array(q_home[:7], dtype=np.float64)

    def set_target_ee_pose(
        self,
        position: np.ndarray,
        quat_xyzw: np.ndarray,
        *,
        max_iters: int = 20,
        tol: float = 1e-4,
        damping: float = 1e-3,
    ) -> None:
        """DLS IK from the current target_q seed to the requested EE pose
        (position in meters, orientation as xyzw quaternion in the URDF frame)."""
        pin = self._pin

        with self._lock:
            q = self._target_q.copy() if self._target_q is not None else np.zeros(7)

        target_R = Rotation.from_quat(quat_xyzw).as_matrix()
        target_pose = pin.SE3(target_R, np.asarray(position, dtype=np.float64))

        n_joints = q.shape[0]
        for _ in range(max_iters):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)
            curr = self._data.oMf[self._ee_frame_id]
            err = pin.log(curr.inverse() * target_pose).vector  # 6D twist in current frame
            if np.linalg.norm(err) < tol and self._null_gain == 0.0:
                break

            J = pin.computeFrameJacobian(
                self._model, self._data, q, self._ee_frame_id, pin.ReferenceFrame.LOCAL
            )
            # Damped pseudo-inverse J^+ = J^T (J J^T + damp I)^{-1}
            J_pinv = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), np.eye(6))
            dq_task = J_pinv @ err

            # Null-space term: project posture error (q_home - q) onto null(J)
            # so it minimizes ||q - q_home||^2 without disturbing the EE task.
            if self._null_gain > 0.0:
                N = np.eye(n_joints) - J_pinv @ J
                dq_null = self._null_gain * N @ (self._q_home - q)
            else:
                dq_null = 0.0

            dq = dq_task + dq_null
            q = q + dq
            if np.linalg.norm(err) < tol and np.linalg.norm(dq_null) < tol:
                break

        with self._lock:
            self._target_q = q

    def get_motor_commands(self, current_time, dt):
        with self._lock:
            if self._target_q is None:
                state = self.get_latest_state()
                self._target_q = np.array(state.x[:7], dtype=np.float64)
            q_des = self._target_q.copy()

        # Feedforward gravity compensation at the measured configuration so the
        # arm holds its pose instead of sagging under its own weight.
        state = self.get_latest_state()
        q_meas = np.array(state.x[:7], dtype=np.float64)
        tau_g = self._pin.computeGeneralizedGravity(self._model, self._data, q_meas)

        for i, cmd in enumerate(self._cmds):
            cmd.desired_position = float(q_des[i])
            cmd.desired_velocity = 0.0
            cmd.torque_ff = float(tau_g[i])
        return self._cmds
