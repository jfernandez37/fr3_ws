import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.logging import LoggingSeverity
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class FindObject(Node):

    def __init__(self):
        super().__init__('find_object')
        self.thresh_image = None
        self.bridge = CvBridge()
        self.cv_image = None
        self.declare_parameter('thresh_value', 160)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def find_correct_contour(self, contours):
        areas = [cv2.contourArea(cnt) for cnt in contours]
        return areas.index(max(areas))
    
    def remove_non_circles(self, contours : tuple):
        new_contours = [cnt for cnt in contours if not cv2.isContourConvex(cnt)]
        return new_contours
                
    
    def listener_callback(self, msg):
        thresh_value = self.get_parameter('thresh_value').get_parameter_value().integer_value
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgra8')#[110:400, 170:600]#y,x
        gray_img = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        gray_img = gray_img
        blurred_img = cv2.blur(gray_img, (5,5))
        _,self.thresh_image = cv2.threshold(blurred_img,thresh_value,255,0)
        contours, _ = cv2.findContours(self.thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(type(contours))
        print(len(contours))
        contours = self.remove_non_circles(contours)
        print(len(contours))
        cv2.drawContours(self.cv_image, contours, -1, (0,255,0), 3)
        cnt = contours[self.find_correct_contour(contours)]
        M = cv2.moments(cnt)
        # print( M )
        try:
            print("m10:",M['m10'],"m01:",M['m01'], "m00:",M['m00'])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # print("x:",cx," y:",cy)
            img_w_center = cv2.circle(self.cv_image, (cx,cy), radius=5, color=(0,0,255), thickness=-1)
        except:
            print("Error: Contour does not form a single shape")
        
    
        