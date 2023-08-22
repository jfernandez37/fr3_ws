#!/usr/bin/env python3
import cv2
import rclpy
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from time import time
    
def point_from_time(t:float, x_vals: list, y_vals: list, times: list):
    x_val = (x_vals[1]-x_vals[0])/(times[1]-times[0])*(t-times[0])+x_vals[0]
    y_val = (y_vals[1]-y_vals[0])/(times[1]-times[0])*(t-times[0])+y_vals[0]
    return (x_val, y_val)
def main(args=None):
    rclpy.init(args=args)
    start_time = time()
    times = []
    x_vals = []
    y_vals = []
    while len(x_vals)<2:
        gear_center_values = [0 for i in range(3)]
        find_object = FindObject()
        rclpy.spin_once(find_object)
        if find_object.ret_cent_gear().count(None) == 0:
            object_depth = ObjectDepth(find_object.ret_cent_gear())
            rclpy.spin_once(object_depth)
            object_depth.destroy_node()
            find_object.destroy_node()
            gear_center_values[0] = object_depth.dist_x
            gear_center_values[1] = object_depth.dist_y
            gear_center_values[2] = object_depth.dist_z
            print(gear_center_values)
            if gear_center_values.count(0)==0:
                times.append(time()-start_time)
                x_vals.append(object_depth.dist_x)
                y_vals.append(object_depth.dist_y)
    for i in range(12):
        print(f"After {i/4} seconds, the gear is ",str(point_from_time(i/4,x_vals, y_vals,times)),"away from the center of the camera")
    print("X_vals:"," ".join([str(val) for val in x_vals]))
    print("times:", " ".join([str(t) for t in times]))
    print("x: " + str(object_depth.dist_x), end="\t")
    print("y: " + str(object_depth.dist_y), end="\t")
    print("z: " + str(object_depth.dist_z) + "\n")
    cv2.imshow("Threshold image", find_object.thresh_image)
    cv2.imshow("Depth image", find_object.cv_image)
    cv2.waitKey(0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
