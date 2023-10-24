#!/usr/bin/env python3
import cv2
import rclpy
from gear_place.multiple_gears import MultipleGears
from gear_place.object_depth import ObjectDepth

def main(args=None):
    rclpy.init(args=args)

    gear_center_target = []
    # while len(gear_center_target)==0:
    gear_center_target = []
    multiple_gears = MultipleGears()
    rclpy.spin_once(multiple_gears)
    while sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0:
        multiple_gears.destroy_node()
        multiple_gears = MultipleGears()
        rclpy.spin_once(multiple_gears)
    object_depth = ObjectDepth(multiple_gears.g_centers)
    rclpy.spin_once(object_depth)  # Gets the distance from the camera
    object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
    # if gear_center_target.count([0,0,0]) != len(gear_center_target):
    #       break
    gear_center_target = object_depth.coordinates
    multiple_gears.destroy_node()
    print(gear_center_target)
    print("None count = ",sum([cent.count(None) for cent in gear_center_target]))
    print(len(gear_center_target))
    cv2.imshow("Threshold image", multiple_gears.thresh_image)
    cv2.imshow("Depth image", multiple_gears.cv_image)
    cv2.waitKey(0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
