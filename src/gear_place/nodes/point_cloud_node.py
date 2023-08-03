#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

import numpy as np


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: PointCloud2):
        # data = read_points_numpy(msg,field_names = ("x", "y", "z"), skip_nans=True, uvs=[[334,349]])
        data = read_points_numpy(msg, reshape_organized_cloud=True, skip_nans=True)
        print(data[434][224])
        # data = read_points(msg,field_names = ("x", "y", "z"), uvs=[437])
        # self.get_logger().info("Data: "+ ", ".join([str(d) for d in data]))
        print(data.shape, end="\n\n")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
