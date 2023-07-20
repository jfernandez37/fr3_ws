#!/usr/bin/env python3
import cv2
import rclpy

from find_object.find_object import FindObject

def main(args=None):
    rclpy.init(args=args)

    find_object = FindObject()

    while find_object.cv_image is None:
        rclpy.spin_once(find_object)
    cv2.imshow("Blurred", find_object.blurred_image)
    cv2.imshow("Thresh", find_object.thresh_image)
    cv2.imshow("Image", find_object.cv_image)
    cv2.waitKey(0)

    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()