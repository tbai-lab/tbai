#!/usr/bin/env python3

import sys
import argparse
import time

import tbai_python

from tbai_python import (
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from joystick import UIController


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
    def __init__(self, ui_controller: UIController):
        super().__init__()
        self.ui_controller = ui_controller

    def getReferenceVelocity(self, time, dt):
        ref = ReferenceVelocity()
        ref.velocity_x = self.ui_controller.linear_x
        ref.velocity_y = self.ui_controller.linear_y
        ref.yaw_rate = self.ui_controller.angular_z
        return ref


def noop_callback(current_time, dt):
    pass


def main():
    parser = argparse.ArgumentParser(description="Deploy NP3O controller on Go2")
    parser.add_argument("--net", type=str, default="eth0",
                        help="Network interface for Unitree SDK (default: eth0)")
    parser.add_argument("--channel", type=int, default=0,
                        help="Unitree channel ID (default: 0)")
    parser.add_argument("--no-lidar", action="store_true",
                        help="Disable lidar subscription")
    args = parser.parse_args()

    if not tbai_python.HAS_DEPLOY_GO2:
        print("Error: tbai_python was built without Go2 support (TBAI_BUILD_DEPLOY_GO2=OFF)")
        sys.exit(1)

    # Create Go2RobotInterface
    robot_args = tbai_python.Go2RobotInterfaceArgs()
    robot_args.set_network_interface(args.net)
    robot_args.set_unitree_channel(args.channel)
    robot_args.set_channel_init(True)
    robot_args.set_subscribe_lidar(not args.no_lidar)

    print(f"Connecting to Go2 on {args.net} (channel {args.channel})...")
    robot = tbai_python.Go2RobotInterface(robot_args)

    print("Waiting for robot to initialize...")
    robot.waitTillInitialized()
    print("Robot initialized.")

    # Set up controller switching
    controller_sub = Go2ChangeControllerSubscriber()

    ui_controller = UIController(
        stand_callback=controller_sub.stand_callback,
        sit_callback=controller_sub.sit_callback,
        bob_callback=controller_sub.bob_callback,
        np3o_callback=controller_sub.np3o_callback,
    )
    ref_vel_gen = Go2ReferenceVelocityGenerator(ui_controller)

    tbai_python.write_init_time()

    # Go2RobotInterface is both a CommandPublisher and StateSubscriber
    central_controller = tbai_python.CentralController.create(robot, controller_sub)

    static_ctrl = tbai_python.StaticController(robot, noop_callback)
    np3o_ctrl = tbai_python.Np3oController(robot, ref_vel_gen, noop_callback)

    central_controller.add_controller(static_ctrl)
    central_controller.add_controller(np3o_ctrl)

    central_controller.startThread()

    try:
        ui_controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping controller...")
        central_controller.stopThread()
        print("Done.")


if __name__ == "__main__":
    main()
