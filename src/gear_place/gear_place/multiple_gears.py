import cv2
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image  # msg for recieving the image

from cv_bridge import (
    CvBridge,
)  # for converting the sensor_msgs.msg.image to opencv image

from math import sqrt

class MultipleGears(Node):
    def __init__(self, connected):
        super().__init__("multiple_gears")
        self.bridge = CvBridge()
        self.cv_image = None
        self.ran = False
        self.g_centers = []
        self.dist_points = {}
        self.connected = connected
        self.thresh_image = None
        self.declare_parameter("thresh_value", 50)
        self.camera_sub = self.create_subscription(
            Image, "/camera/color/camera_info", self.camera_cb, 1
        )
        self.camera_sub
        self.subscription = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def camera_cb(self, msg: Image):
        if not self.connected:
            self.get_logger().info("\n\n\n==============Camera is connected==============\n\n")
            self.connected = True
        
    def closest_to_circle(self, contours : list) -> list:
        """
        Returns the contour which is closest to a circle
        """
        circle_areas = []
        areas = [cv2.contourArea(cnt) for cnt in contours]
        for cnt in contours:
            (_, _), radius = cv2.minEnclosingCircle(cnt)
            circle_areas.append(__import__("math").pi * radius**2)
        diffs = [areas[i] / circle_areas[i] for i in range(len(areas))]
        gears = [i for i in range(len(diffs)) if diffs[i] >= 0.9]
        return sorted(list(set(gears)))

    def remove_bad_contours(self, contours: tuple) -> list:
        """
        Removes contours which are too small and ones with too few sides to be the gear
        """
        minimum_contour_area = 1500
        return [cnt  for cnt in contours if cv2.contourArea(cnt) > minimum_contour_area and not cv2.isContourConvex(cnt)]

    def listener_callback(self, msg : Image):
        """
        Gets the image from the contour, blurs it, applies a threshold, finds the contours.
        Then, the functions above are used to find the gear out of all the contours that are found.
        It then finds the center of the gear contour.
        """
        # TODO
        # while not self.connected:
        #     self.get_logger().info("Camera not connected yet. Waiting until ready")
        #     __import__("time").sleep(3)
        self.ran = True
        min_thresh, max_thresh = 10, 180
        thresh_value = (
            self.get_parameter("thresh_value").get_parameter_value().integer_value
        )
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        cv_image_array = np.array(cv_image, dtype=np.dtype("u1"))
        self.cv_image = cv_image_array
        alpha = 2.5  # Contrast control (1.0-3.0)
        beta = -65  # Brightness control (-100-100)
        self.cv_image = cv2.convertScaleAbs(self.cv_image, alpha=alpha, beta=beta)
        for i in range(len(self.cv_image)):
            for j in range(len(self.cv_image[i])):
                if self.cv_image[i][j] == 255:
                    self.cv_image[i][j] = 0
        blurred_img = cv2.GaussianBlur(self.cv_image, (7,7),0)
        for _ in range(3):
            blurred_img = cv2.GaussianBlur(blurred_img, (7, 7), 0)
        valid_contours = []
        self.get_logger().info("Starting scan")
        for i in range(min_thresh, max_thresh + 1,3):
            thresh_value = i
            _, self.thresh_image = cv2.threshold(
                blurred_img, thresh_value, 255, cv2.THRESH_BINARY_INV
            )
            contours, _ = cv2.findContours(
                self.thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            contours = self.remove_bad_contours(contours)
            if len(contours) >= 1:
                for ind in self.closest_to_circle(contours):
                    valid_contours.append(ind)
                    (x, y), radius = cv2.minEnclosingCircle(contours[ind])
                    radius = int(radius)
                    if (int(x), int(y)) not in self.g_centers:
                        self.g_centers.append((int(x), int(y)))
                        unit_circle = sqrt(2)/2
                        between_x_y = int(unit_circle*radius)
                        self.dist_points[(int(x), int(y))] = []
                        if int(x)+radius <=640:
                            self.dist_points[(int(x), int(y))].append((int(x)+radius,int(y)))
                        if int(x)-radius >=0:
                            self.dist_points[(int(x), int(y))].append((int(x)-radius,int(y)))
                        if int(y)+radius <= 480:
                            self.dist_points[(int(x), int(y))].append((int(x),int(y)+radius))
                        if int(y)-radius >=0:
                            self.dist_points[(int(x), int(y))].append((int(x),int(y)-radius))

                        self.dist_points[(int(x), int(y))]+=[(int(x)+between_x_y*[-1,1][i],int(y)+between_x_y*[-1,1][j]) for i in range(2) for j in range(2) if 0<=int(x)+between_x_y*[-1,1][i]<=640 and 0<=int(y)+between_x_y*[-1,1][i]<=480]
                        
        if len(valid_contours) == 0:
            return

class MultipleGearsHigh(Node):
    def __init__(self, connected):
        super().__init__("multiple_gears_high")
        self.bridge = CvBridge()
        self.cv_image = None
        self.ran = False
        self.g_centers = []
        self.dist_points = {}
        self.connected = connected
        self.thresh_image = None
        self.declare_parameter("thresh_value", 50)
        self.camera_sub = self.create_subscription(
            Image, "/camera/color/camera_info", self.camera_cb, 1
        )
        self.camera_sub
        self.subscription = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def camera_cb(self, msg: Image):
        if not self.connected:
            self.get_logger().info("\n\n\n==============Camera is connected==============\n\n")
            self.connected = True
        
    def closest_to_circle(self, contours : list) -> list:
        """
        Returns the contour which is closest to a circle
        """
        circle_areas = []
        areas = [cv2.contourArea(cnt) for cnt in contours]
        for cnt in contours:
            (_, _), radius = cv2.minEnclosingCircle(cnt)
            circle_areas.append(__import__("math").pi * radius**2)
        diffs = [areas[i] / circle_areas[i] for i in range(len(areas))]
        gears = [i for i in range(len(diffs)) if diffs[i] >= 0.9]
        return sorted(list(set(gears)))

    def remove_bad_contours(self, contours: tuple) -> list:
        """
        Removes contours which are too small and ones with too few sides to be the gear
        """
        minimum_contour_area = 200
        return [cnt  for cnt in contours if cv2.contourArea(cnt) > minimum_contour_area and not cv2.isContourConvex(cnt)]

    def listener_callback(self, msg : Image):
        """
        Gets the image from the contour, blurs it, applies a threshold, finds the contours.
        Then, the functions above are used to find the gear out of all the contours that are found.
        It then finds the center of the gear contour.
        """
        # TODO
        # while not self.connected:
        #     self.get_logger().info("Camera not connected yet. Waiting until ready")
        #     __import__("time").sleep(3)
        self.ran = True
        min_thresh, max_thresh = 0, 255
        thresh_value = (
            self.get_parameter("thresh_value").get_parameter_value().integer_value
        )
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        cv_image_array = np.array(cv_image, dtype=np.dtype("u1"))
        self.cv_image = cv_image_array
        alpha = 2.5  # Contrast control (1.0-3.0)
        beta = -65  # Brightness control (-100-100)
        self.cv_image = cv2.convertScaleAbs(self.cv_image, alpha=alpha, beta=beta)
        for i in range(len(self.cv_image)):
            for j in range(len(self.cv_image[i])):
                if self.cv_image[i][j] == 255:
                    self.cv_image[i][j] = 0
        blurred_img = cv2.GaussianBlur(self.cv_image, (7,7),0)
        for _ in range(3):
            blurred_img = cv2.GaussianBlur(blurred_img, (7, 7), 0)
        valid_contours = []
        self.get_logger().info("Starting scan")
        for i in range(min_thresh, max_thresh + 1):
            thresh_value = i
            _, self.thresh_image = cv2.threshold(
                blurred_img, thresh_value, 255, cv2.THRESH_BINARY_INV
            )
            contours, _ = cv2.findContours(
                self.thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            contours = self.remove_bad_contours(contours)
            if len(contours) >= 1:
                for ind in self.closest_to_circle(contours):
                    valid_contours.append(ind)
                    (x, y), radius = cv2.minEnclosingCircle(contours[ind])
                    radius = int(radius)
                    if (int(x), int(y)) not in self.g_centers:
                        self.g_centers.append((int(x), int(y)))
                        unit_circle = sqrt(2)/2
                        between_x_y = int(unit_circle*radius)
                        self.dist_points[(int(x), int(y))] = []
                        if int(x)+radius <=640:
                            self.dist_points[(int(x), int(y))].append((int(x)+radius,int(y)))
                        if int(x)-radius >=0:
                            self.dist_points[(int(x), int(y))].append((int(x)-radius,int(y)))
                        if int(y)+radius <= 480:
                            self.dist_points[(int(x), int(y))].append((int(x),int(y)+radius))
                        if int(y)-radius >=0:
                            self.dist_points[(int(x), int(y))].append((int(x),int(y)-radius))

                        self.dist_points[(int(x), int(y))]+=[(int(x)+between_x_y*[-1,1][i],int(y)+between_x_y*[-1,1][j]) for i in range(2) for j in range(2) if 0<=int(x)+between_x_y*[-1,1][i]<=640 and 0<=int(y)+between_x_y*[-1,1][i]<=480]
                        
        if len(valid_contours) == 0:
            return