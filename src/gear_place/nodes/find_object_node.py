#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
def main(args=None):
    rclpy.init(args=args)

    gear_center_values = [0 for i in range(3)]
    while(gear_center_values.count(0)==3):
        find_object = FindObject()
        #rclpy.spin(find_object)
        rclpy.spin_once(find_object)
        cv2.imshow("Threshold image", find_object.thresh_image)
        cv2.imshow("Depth image", find_object.cv_image)
        # cv2.imshow("Originial image", find_object.original_image)
        cv2.waitKey(0)
        # __import__("time").sleep(1)
        object_depth = ObjectDepth(find_object.ret_cent_gear())
        rclpy.spin_once(object_depth)
        
        object_depth.destroy_node()
        find_object.destroy_node()
        gear_center_values[0] = object_depth.dist_x
        gear_center_values[1] = object_depth.dist_y
        gear_center_values[2] = object_depth.dist_z
        print(gear_center_values)
    print("x: "+str(object_depth.dist_x),end="\t")
    print("y: "+str(object_depth.dist_y),end="\t")
    print("z: "+str(object_depth.dist_z)+"\n")
    rclpy.shutdown()


if __name__ == '__main__':
    main()