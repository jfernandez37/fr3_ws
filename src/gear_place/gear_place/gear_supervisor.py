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

from transform_utils import (
    multiply_pose,
    convert_transform_to_pose,
    euler_from_quaternion,
    quaternion_from_euler,
    transform_from_pose,
    build_pose,
)

from gmcs_interfaces.msg import AssemblyObject
from gmcs_interfaces.srv import (
    AddObjectToPlanningScene,
    GetFiducialPose,
    MoveObjectToBoard,
    MoveToNamedPose,
    MoveToPosition,
    PickObject,
    AssembleObject
)

class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return(repr(self.value))

class GearSupervisor(Node):
    def __init__(self):
        super().__init__('assembly_supervisor')

        # Read transforms file
        self.declare_parameter('transforms_config', "")
        transforms_path = self.get_parameter('transforms_config').get_parameter_value().string_value

        if transforms_path == "":
            raise Error("Transforms config parameter not set")
        
        with open(transforms_path, "r") as stream:
            try:
                self.transforms = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                raise Error("unable to open transforms config")      

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_transforms = []

        # Subscribers
        self.ee_pose : Pose
        self.ee_pose = None
        self.ee_pose_sub = self.create_subscription(PoseStamped, '/ee_pose', self._ee_pose_cb, 10)

        # Service clients
        self.find_object_client = self.create_client(FindObject, "find_object")

    def wait(self, duration: float):
        # self.get_logger().info(f"Waiting for {duration} seconds...")

        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self, timeout_sec=0)
                # self.get_logger().log(str(self.get_clock().now().to_msg().sec), LoggingSeverity.INFO, throttle_duration_sec=1.0)
            except KeyboardInterrupt:
                raise Error("Ctrl+C pressed")

    def move_to_target(self, x, y, z):
        target_point = Point()
        target_point.x = x
        target_point.y = y
        target_point.z = z

        try:
            self._call_move_to_position_service(target_point)
        except Error as e:
            raise e
                

    def _call_move_to_named_pose_service(self, named_pose: str):
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
        
    def _call_move_to_position_service(self, p: Point, rot: float = 0.0):
        self.get_logger().info(f"Moving to position ({p.x}, {p.y}, {p.z})")

        request = MoveToPosition.Request()
        request.target_position = p
        request.gripper_rotation = rot

        future = self.move_to_position_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=8)

        if not future.done():
            raise Error("Timeout reached when calling move_to_position service")

        result : MoveToPosition.Response
        result = future.result()

        if not result.success:
            raise Error(f"Unable to move to desired position [{p.x}, {p.y}, {p.z}]")
        
    def _ee_pose_cb(self, msg: PoseStamped):
        self.ee_pose = msg.pose
    
    def _calculate_world_pose(self, frame_id: str, rel_pose: Pose) -> Pose:
        # Lookup transform from world to frame_id
        try:
            t = self.tf_buffer.lookup_transform('world', frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {frame_id} to world: {ex}')
            raise Error("Unable to transform between frames")
        
        return multiply_pose(convert_transform_to_pose(t), rel_pose)
