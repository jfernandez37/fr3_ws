import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.logging import LoggingSeverity
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import yaml
import math
from typing import List, Tuple

from geometry_msgs.msg import Pose, Point, PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from gear_place.transform_utils import(
    multiply_pose,
    convert_transform_to_pose,
    euler_from_quaternion,
    quaternion_from_euler,
    transform_from_pose,
    build_pose,
)

from gear_place_interfaces.srv import (
    MoveCartesian,
    MoveToNamedPose,
)

class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return(repr(self.value))

class GearPlace(Node):
    def _init__(self):
        super().__init__("gear_place")
        
        #Service Clients
        self.move_to_named_pose_client = self.create_client(MoveToNamedPose, 'move_to_named_pose')
        self.move_cartesian_client = self.create_client(MoveCartesian, "move_cartesian")
    
    def wait(self, duration: float):
        # self.get_logger().info(f"Waiting for {duration} seconds...")

        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self, timeout_sec=0)
                # self.get_logger().log(str(self.get_clock().now().to_msg().sec), LoggingSeverity.INFO, throttle_duration_sec=1.0)
            except KeyboardInterrupt:
                raise Error("Ctrl+C pressed")
    
    def _call_move_to_named_pose_service(self, named_pose: str):
        '''
        Calls the move_to_named_pose callback
        '''
        self.get_logger().info(f"Moving to {named_pose}")

        request = MoveToNamedPose.Request()

        request.pose = named_pose

        future = self.move_to_named_pose_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling move_to_named_pose service")

        result : MoveToNamedPose.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f'Unable to move to pose: {named_pose}')
            raise Error("Unable to move to pose")
    
    def _move_cartesian_service(self, x,y,z,v_max, acc):
        '''
        Calls the move_cartesian callback
        '''
        self.get_logger().info(f"Moving {x},{y},{z}")
        
        request = MoveCartesian.Request()
        
        request.x = x
        request.y = y
        request.z = z
        request.max_velocity = v_max
        request.acceleration = acc
        
        future = self.move_cartesian_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        
        if not future.done():
            raise Error("Timeout reached when calling move_cartesian service")
        
        result : MoveCartesian.Response
        result = future.result()
        
        if not result.success:
            self.get_logger().error(f"Unable to move {x},{y},{z}")
            raise Error("Unable to move to location")