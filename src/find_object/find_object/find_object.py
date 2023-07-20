import cv2
import rclpy
from rclpy.node import Node
# from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class FindObject(Node):

    def __init__(self):
        super().__init__('find_object')
        self.thresh_image = None
        self.blurred_image = None
        self.bridge = CvBridge()
        self.cv_image = None
        self.declare_parameter('thresh_value', 160)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def closest_to_circle(self, contours):
        circle_areas = []
        areas = [cv2.contourArea(cnt) for cnt in contours]
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            circle_areas.append(__import__("math").pi * radius**2)
        diffs = [circle_areas[i]-areas[i] for i in range(len(areas))]
        print(diffs)
        return diffs.index(min(diffs))
            
    def find_correct_contour(self, contours):
        areas = [cv2.contourArea(cnt) for cnt in contours]
        return areas.index(max(areas))
    
    def remove_bad_contours(self, contours : tuple):
        minimum_area = 400
        new_contours = [cnt for cnt in contours if not cv2.isContourConvex(cnt)]
        filtered_contours=[]
        for cnt in new_contours:
            epsilon = 0.01*cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx)>10 and cv2.contourArea(cnt)>=minimum_area:
                filtered_contours.append(cnt)
        return filtered_contours
                
    
    def listener_callback(self, msg):
        thresh_value = self.get_parameter('thresh_value').get_parameter_value().integer_value
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgra8')#[110:400, 170:600]#y,x
        gray_img = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.GaussianBlur(gray_img,(7,7),0)
        self.blurred_image = blurred_img
        _,self.thresh_image = cv2.threshold(blurred_img,thresh_value,255,0)
        contours, _ = cv2.findContours(self.thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        before_remove = len(contours)
        contours = self.remove_bad_contours(contours)
        print(before_remove-len(contours), " contours were removed")
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(self.cv_image,center,radius,(0,255,0),2)
        cv2.drawContours(self.cv_image, contours, -1, (0,255,0), 3)
        # M = cv2.moments(contours[self.find_correct_contour(contours)]) # Finds contour with largest area
        M = cv2.moments(contours[self.closest_to_circle(contours)]) #Finds the contour that is closest to a circle
        # print( M )
        try:
            print("m10:",M['m10'],"m01:",M['m01'], "m00:",M['m00'])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # print("x:",cx," y:",cy)
            img_w_center = cv2.circle(self.cv_image, (cx,cy), radius=10, color=(0,0,255), thickness=-1)
        except:
            print("Error: Contour does not form a single shape")
        
    
        