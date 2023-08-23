#!/usr/bin/env python3

import rclpy
from gear_place.gear_place_classes import GearPlace, Error, ConveyorClass

from time import sleep


def main(args=None):
    rclpy.init(args=args)

    try:
        supervisor = GearPlace("gear_place")
        conveyor_supervisor = ConveyorClass("aprs_ros_conveyor")
        supervisor.wait(0.75)
        supervisor._call_move_to_named_pose_service("home")  # starts in the home position
        supervisor._call_move_cartesian_service(-0.27, 0.0, 0.0, 0.15, 0.2)  # Moves to the center of the cart
        supervisor._call_pick_up_gear_service(0.0095)  # Moves to above the gear, opens the gripper to the maximum, then down to the gear, grabs the gear, then picks it up
        supervisor._call_move_to_named_pose_service("home")  # starts in the home position
        supervisor._call_open_gripper_service() # opens the gripper so that time delay is easier to calculate on moving gear pick up 
        supervisor._call_put_gear_down_service()  # moves down, releases the gear, and moves back up
        """
        supervisor._call_move_to_named_pose_service("home")
        supervisor._call_move_to_conveyor_service(-0.27, -0.3, z_movement + 0.1)
        conveyor_supervisor._enable_conveyor_service(True)
        conveyor_supervisor._set_conveyor_state_service(speed=50, direction=0)
        conveyor_supervisor._enable_conveyor_service(False)
        """

    except Error as e:
        print(e)


if __name__ == "__main__":
    main()
