#!/usr/bin/env python3

import sys
import dataclasses
import time

import tyro

import tbai as tbai_python

from tbai import (
    RobotInterface,
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from tbai import rotations

from joystick import UIController

import os
import tbai_descriptions

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
    robot_args.set_network_interface(args.net)
    robot_args.set_unitree_channel(args.channel)
    robot_args.set_channel_init(True)
    robot_args.set_subscribe_lidar(not args.no_lidar)
    robot_args.set_enable_video(args.video)

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
