from rclpy.node import Node
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from time import time
import rclpy
from math import sqrt


class MovingGear:
    def __init__(self):
        self.times = []
        self.x_vals = []
        self.y_vals = []
        self.x_pix = []
        self.y_pix = []
        self.start_time = time()
        self.found_gear = False

    def run(self):
        self.found_gear = False
        c = 0
        while len(self.x_vals) < 2:
            c += 1
            gear_center_values = [0]
            find_object = FindObject()
            rclpy.spin_once(find_object)
            if find_object.ret_cent_gear().count(None) == 0:
                object_depth = ObjectDepth([(find_object.ret_cent_gear())],{})
                rclpy.spin_once(object_depth)
                object_depth.destroy_node()
                find_object.destroy_node()
                for coord in object_depth.coordinates:
                    gear_center_values[0] = coord[2]
                print(gear_center_values)
                if gear_center_values.count(0) == 0:
                    x,y = find_object.ret_cent_gear()
                    self.x_pix.append(x)
                    self.y_pix.append(y)
                    self.times.append(time() - self.start_time)
                    self.x_vals.append(coord[0])
                    self.y_vals.append(coord[1])
                    self.found_gear = True
                object_depth.destroy_node()
            find_object.destroy_node()
            if c % 6 == 0:
                self.x_vals = []
                self.y_vals = []
                self.times = []
                return

    def point_from_time(self, t: float):
        x_val = (self.x_vals[1] - self.x_vals[0]) / (self.times[1] - self.times[0]) * (
            t - self.times[1]
        ) + self.x_vals[1]
        y_val = (self.y_vals[1] - self.y_vals[0]) / (self.times[1] - self.times[0]) * (
            t - self.times[1]
        ) + self.y_vals[1]
        return (x_val, y_val)

    def distance_to_point(self, p: tuple):
        return sqrt(sum([v**2 for v in p]))

    def distance_formula(self):
        time_vals = []
        distances = []
        for i in range(1, 3):
            time_vals.append(i)
            distances.append(self.distance_to_point(self.point_from_time(i)))
        slope = (distances[1] - distances[0]) / (time_vals[1] - time_vals[0])
        intercept = distances[1] - (time_vals[1] * slope)
        return slope, intercept
