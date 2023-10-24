#!/usr/bin/env python3

import rclpy
from gear_place.gear_place_classes import GearPlace, Error
from time import sleep
from math import pi

def main(args=None):
    rclpy.init(args=args)
    gear_width = 0.0095
    try:
        supervisor = GearPlace()
        supervisor.wait(5)
        # supervisor._call_open_gripper_service()
        # supervisor._call_move_to_named_pose_service("home")
        # supervisor._call_move_to_named_pose_service("high_scan")
        # supervisor._call_pick_up_gear_service(gear_width)
        # supervisor._call_multiple_gears_single_scan(gear_width)
        # supervisor._call_rotate_single_joint(6,90.0,False)
        # sleep(1.0)
        # supervisor._call_rotate_single_joint(6,-pi/2,True)
        # supervisor._call_move_cartesian_smooth_service(-0.27,0.0,0.0,0.3,0.3)
        # supervisor._call_multiple_gears_single_scan(gear_width)
        gear_positions, radius_vals = supervisor.scan_multiple_gears_grid()
        supervisor.pick_up_multiple_gears_depth(gear_positions,radius_vals,gear_width, "home")
        # supervisor._call_move_to_named_pose_service("above_conveyor")
        # supervisor.wait(2) 
        # while True:
        #     supervisor._call_pick_up_moving_gear_service(gear_width, True)  # Moves to above the gear, opens the gripper to the maximum, then down to the gear, grabs the gear, then picks it up
        #     supervisor._call_move_to_named_pose_service("home")
        #     supervisor._call_put_gear_down_camera(-0.5)
        #     supervisor._call_move_to_named_pose_service("above_conveyor")


        supervisor._enable_conveyor_service(True)
        supervisor._set_conveyor_state_service(speed=50, direction=0)
        sleep(10)
        supervisor._enable_conveyor_service(False)

    except Error as e:
        print(e)


if __name__ == "__main__":
    main()
