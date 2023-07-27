#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
def main(args=None):
    rclpy.init(args=args)

    find_object = FindObject()
    # rclpy.spin(find_object)
    while True:
        rclpy.spin_once(find_object)
        # __import__("time").sleep(1)
        object_depth = ObjectDepth(find_object.ret_cent_gear())
        rclpy.spin_once(object_depth)
        print("x: "+str(object_depth.dist_x),end="\t")
        print("y: "+str(object_depth.dist_y),end="\t")
        print("z: "+str(object_depth.dist_z)+"\n")
        # cv2.imshow("Blurred", find_object.blurred_image)
        # cv2.imshow("Thresh", find_object.thresh_image)
        # cv2.imshow("Image", find_object.cv_image)
        __import__("time").sleep(3)

    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()