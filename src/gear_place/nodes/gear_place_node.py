#!/usr/bin/env python3

import rclpy
from gear_place.gear_place_classes import GearPlace, Error, ConveyorClass
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from time import sleep


def main(args=None):
    gear_center_target = [0 for i in range(3)]
    rclpy.init(args=args)

    try:
        x_offset = 0.047  # offset from the camera to the gripper
        y_offset = 0.03  # offset from the camera to the gripper
        z_movement = (
            -0.247
        )  # z distance from the home position to where the gripper can grab the gear
        supervisor = GearPlace("gear_place")
        conveyor_supervisor = ConveyorClass("aprs_ros_conveyor")
        supervisor.wait(0.75)
        supervisor._call_move_to_named_pose_service(
            "home"
        )  # starts in the home position
        sleep(3)
        supervisor._call_move_cartesian_service(
            -0.27, 0.0, 0.0, 0.15, 0.2
        )  # Moves to the center of the cart
        sleep(1)

        
        supervisor._call_pick_up_gear_service(
            0.0095,
        )  # Moves to above the gear, opens the gripper to the maximum, then down to the gear, grabs the gear, then picks it up
        sleep(1.5)
        supervisor._call_put_gear_down_service(
            z_movement
        )  # moves down, releases the gear, and moves back up
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
