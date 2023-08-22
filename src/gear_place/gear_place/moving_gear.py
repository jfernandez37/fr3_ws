from rclpy.node import Node
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from time import time
import rclpy

class MovingGear():
    def __init__(self):
        self.times = []
        self.x_vals = []
        self.y_vals = []
        self.start_time = time()
        while len(self.x_vals)<2:
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
                    self.times.append(time()-self.start_time)
                    self.x_vals.append(object_depth.dist_x)
                    self.y_vals.append(object_depth.dist_y)
        
    def point_from_time(self, t:float):
        x_val = (self.x_vals[1]-self.x_vals[0])/(self.times[1]-self.times[0])*(t-self.times[0])+self.x_vals[0]
        y_val = (self.y_vals[1]-self.y_vals[0])/(self.times[1]-self.times[0])*(t-self.times[0])+self.y_vals[0]
        return (x_val, y_val)    