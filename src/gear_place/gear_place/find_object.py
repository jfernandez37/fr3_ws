import cv2
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image  # msg for recieving the image

from cv_bridge import (
    CvBridge,
)  # for converting the sensor_msgs.msg.image to opencv image


class FindObject(Node):
    def __init__(self):
        super().__init__("find_object")
        self.bridge = CvBridge()
        self.cv_image = None
        self.gx = None
        self.gy = None
        self.thresh_image = None
        self.declare_parameter("thresh_value", 50)
        self.subscription = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.listener_callback, 1
        )
        self.subscription  # prevent unused variable warning

    def closest_to_circle(self, contours):
        """
        Returns the contour which is closest to a circle
        """
        circle_areas = []
        areas = [cv2.contourArea(cnt) for cnt in contours]
        for cnt in contours:
            (_, _), radius = cv2.minEnclosingCircle(cnt)
            circle_areas.append(__import__("math").pi * radius**2)
        diffs = [areas[i] / circle_areas[i] for i in range(len(areas))]
        return diffs.index(max(diffs)), diffs[diffs.index(max(diffs))]

    def remove_bad_contours(self, contours: tuple):
        """
        Removes contours which are too small and ones with too few sides to be the gear
        """
        minimum_contour_area = 4000
        maximum_contour_area = 7000
        new_contours = [cnt for cnt in contours if not cv2.isContourConvex(cnt)]
        filtered_contours = []
        for cnt in new_contours:
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if (
                len(approx) > 10
                and cv2.contourArea(cnt) > minimum_contour_area
                and maximum_contour_area > cv2.contourArea(cnt)
            ):
                filtered_contours.append(cnt)
        return filtered_contours

    def listener_callback(self, msg):
        """
        Gets the image from the contour, blurs it, applies a threshold, finds the contours.
        Then, the functions above are used to find the gear out of all the contours that are found.
        It then finds the center of the gear contour.
        """
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
                if self.cv_image[i][j] < 10 or 180 < self.cv_image[i][j]:
                    self.cv_image[i][j] = 255
        c = 0
        for i in range(len(self.cv_image)):
            for j in range(len(self.cv_image[i])):
                if self.cv_image[i][j] != 255:
                    for i in range(10):
                        c += 1
                        self.cv_image[i][j] -= 50
                        self.cv_image[i][j] = min(255, self.cv_image[i][j] * 10)
        self.original_image = self.cv_image.copy()
        blurred_img = cv2.GaussianBlur(self.cv_image, (7, 7), 0)
        for i in range(3):
            blurred_img = cv2.GaussianBlur(blurred_img, (7, 7), 0)
        contours_left = 0
        up_down = 1
        c = 0
        contour_to_circle_ratio = 0.0
<<<<<<< HEAD
        while contours_left < 1 or contour_to_circle_ratio<=0.5:
=======
        while contours_left < 1 or contour_to_circle_ratio<=0.75:
>>>>>>> fd58e617d3a1ae155150c8c273663ad8dd44fb86
            c += 1
            _, self.thresh_image = cv2.threshold(
                blurred_img, thresh_value, 255, cv2.THRESH_BINARY_INV
            )
            contours, _ = cv2.findContours(
                self.thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            self.get_logger().info(f"Contours found: {len(contours)}")
            before_remove = len(contours)
            contours = self.remove_bad_contours(contours)
            contours_left = len(contours)
            if thresh_value == 255:
                up_down = -1
            elif thresh_value == 0:
                up_down = 1
            thresh_value += up_down
            if c>=255:
                return
            if contours_left>=1:
                closest_to_circle, contour_to_circle_ratio = self.closest_to_circle(contours)
                print(contour_to_circle_ratio)
        self.get_logger().info(
            f"{before_remove - len(contours)} contours were removed.\n\t\t\t\t\t     Took {c} different "
            + ("threshold" if c == 1 else "thresholds")
        )
        cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 3)
        (x, y), self.radius = cv2.minEnclosingCircle(
            contours[closest_to_circle]
        )
        self.get_logger().info(
            f"{cv2.contourArea(contours[closest_to_circle])}"
        )
        self.gx = int(x)
        self.gy = int(y)
        cv2.circle(
            self.cv_image, (self.gx, self.gy), int(self.radius), (255, 255, 255), 2
        )
        try:
            (h, w) = self.cv_image.shape[:2]
            self.cx = w // 2
            self.cy = h // 2
            cv2.circle(self.cv_image, (w // 2, h // 2), 7, (255, 255, 255), -1)
            cv2.circle(
                self.cv_image,
                (self.gx, self.gy),
                10,
                color=(255, 255, 255),
                thickness=-1,
            )
        except:
            self.get_logger.error("Error: Contour does not form a single shape")
        self.get_logger().info(
            f"X coordinate for gear: {self.gx}, y coordinate for gear {self.gy}"
        )

    def ret_cent_gear(self):
        """
        Returns the x and y coordinates of the pixel at the center of the gear
        """
        return self.gx, self.gy
