#!/usr/bin/env python3

import rclpy
from gear_place.gear_place_classes import GearPlace, Error
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
def main(args=None):
    rclpy.init(args=args)
    
    try:
        supervisor = GearPlace()
        supervisor.wait(2.0)
        supervisor._call_move_to_named_pose_service("home")
        find_object = FindObject()
        rclpy.spin_once(find_object)
        object_depth = ObjectDepth(find_object.ret_cent_gear())
        rclpy.spin_once(object_depth)
        supervisor._pick_up_gear_service(object_depth.dist_x,object_depth.dist_y,object_depth.dist_z,0.01)
        
    except Error as e:
        print(e)

if __name__ == '__main__':
    main()