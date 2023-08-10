#!/usr/bin/env python3

import rclpy
from gear_place.gear_place_classes import GearPlace, Error
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from time import sleep
import cv2

def main(args=None):
    gear_center_target = [0 for i in range(3)]
    rclpy.init(args=args)

    try:
        x_offset = 0.05
        y_offset = 0.05
        z_offset = 0.1
        supervisor = GearPlace("gear_place")
        supervisor.wait(2.0)
        supervisor._call_move_to_named_pose_service("home")
        sleep(3)
        supervisor._call_move_cartesian_service(-0.20, 0.0, 0.0, 0.15, 0.2)
        sleep(1)
        
        while gear_center_target.count(0) == 3 or None in gear_center_target:
            find_object = FindObject()
            rclpy.spin_once(find_object)
            object_depth = ObjectDepth(find_object.ret_cent_gear())
            rclpy.spin_once(object_depth)
            object_depth.destroy_node()
            find_object.destroy_node()
            gear_center_target[0] = object_depth.dist_x
            gear_center_target[1] = object_depth.dist_y
            gear_center_target[2] = object_depth.dist_z
            sleep(1)
        sleep(3)
        print(gear_center_target)
        cv2.imshow("Depth image", find_object.cv_image)
        supervisor._call_move_cartesian_service(-1*object_depth.dist_y+x_offset,-1*object_depth.dist_x+y_offset
                                                -1*object_depth.dist_z+z_offset,0.15,0.2)
        #     print(gear_center_target)
        # supervisor._call_pick_up_gear_service(
        #     object_depth.dist_x, object_depth.dist_y, object_depth.dist_z, 0.01
        # )
        # supervisor._call_move_to_named_pose_service("home")
        # supervisor._call_move_to_conveyor_service(0.1,0.1,0.1)

    except Error as e:
        print(e)


if __name__ == "__main__":
    main()
