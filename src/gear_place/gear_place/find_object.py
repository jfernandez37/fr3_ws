import cv2
import rclpy
from rclpy.node import Node
# from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class FindObject(Node):

    def __init__(self):
        super().__init__('find_object')
        self.thresh_image = None
        self.invert_thresh = None
        self.blurred_image = None
        self.bridge = CvBridge()
        self.cv_image = None
        self.declare_parameter('thresh_value', 30)
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def closest_to_circle(self, contours):
        circle_areas = []
        areas = [cv2.contourArea(cnt) for cnt in contours]
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            circle_areas.append(__import__("math").pi * radius**2)
        diffs = [areas[i]/circle_areas[i] for i in range(len(areas))]
        return diffs.index(max(diffs))
    
    def remove_bad_contours(self, contours : tuple):
        minimum_contour_area = 500
        new_contours = [cnt for cnt in contours if not cv2.isContourConvex(cnt)]
        filtered_contours=[]
        for cnt in new_contours:
            epsilon = 0.01*cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx)>10 and cv2.contourArea(cnt)> minimum_contour_area:
                filtered_contours.append(cnt)
        return filtered_contours
                
    
    def listener_callback(self, msg):
        thresh_value = self.get_parameter('thresh_value').get_parameter_value().integer_value
        # self.get_logger().info(f"Size of the image {self.cv_image.shape}")
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        cv_image_array = np.array(cv_image, dtype = np.dtype('u1'))
        self.cv_image = cv_image_array
        blurred_img = cv2.GaussianBlur(self.cv_image,(7,7),0)
        _,self.thresh_image = cv2.threshold(blurred_img,thresh_value,255,cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(self.thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Contours found:",len(contours))
        before_remove = len(contours)
        contours = self.remove_bad_contours(contours)
        print(before_remove-len(contours), " contours were removed")            
        cv2.drawContours(self.cv_image, contours, -1, (0,255,0), 3)
        M = cv2.moments(contours[self.closest_to_circle(contours)]) #Finds the contour that is closest to a circle
        (x,y),self.radius = cv2.minEnclosingCircle(contours[self.closest_to_circle(contours)])
        (x,y),radius = cv2.minEnclosingCircle(contours[self.closest_to_circle(contours)])
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(self.cv_image,center,radius,(255,255,255),2)
        # print( M )
        try:
            (h, w) = self.cv_image.shape[:2]
            self.cx = w//2
            self.cy = h//2
            cv2.circle(self.cv_image, (w//2, h//2), 7, (255, 255, 255), -1)
            self.gx = int(M['m10']/M['m00'])
            self.gy = int(M['m01']/M['m00'])
            cv2.circle(self.cv_image, (self.gx,self.gy), 10, color=(255,255,255), thickness=-1)
            # cv2.circle(self.cv_image, (self.gx+int(radius),self.gy), 10, color=(255,255,255), thickness=-1)
        except:
            print("Error: Contour does not form a single shape")
        self.get_logger().info(f"X coordinate for gear: {self.gx}, y coordinate for gear {self.gy}")
        

    def ret_cent_gear(self):
        return self.gx, self.gy
        
    
        