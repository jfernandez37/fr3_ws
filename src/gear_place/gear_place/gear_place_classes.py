import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from gear_place_interfaces.srv import (
    MoveCartesian,
    MoveToNamedPose,
    PickUpGear,
    MoveToConveyor,
    MoveToPosition,
)

from gear_place.transform_utils import multiply_pose, convert_transform_to_pose

from geometry_msgs.msg import Pose, Point


class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return repr(self.value)


class GearPlace(Node):
    def _init__(self):
        # super().__init__("gear_place")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_transforms = []
        print("CONSTRUCTOR RUNNING TEST")
        # Service Clients
        self.move_to_named_pose_client = self.create_client(
            MoveToNamedPose, "move_to_named_pose"
        )
        self.move_cartesian_client = self.create_client(MoveCartesian, "move_cartesian")
        self.pick_up_gear_client = self.create_client(PickUpGear, "pick_up_gear")
        self.move_to_conveyor_client = self.create_client(
            MoveToConveyor, "move_to_conveyor"
        )
        self.move_to_position_client = self.create_client(
            MoveToPosition, "move_to_position"
        )

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
        """
        Calls the move_to_named_pose callback
        """
        self.get_logger().info(f"Moving to {named_pose}")

        request = MoveToNamedPose.Request()

        request.pose = named_pose

        future = self.create_client(MoveToNamedPose, "move_to_named_pose").call_async(
            request
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling move_to_named_pose service")

        result: MoveToNamedPose.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to move to pose: {named_pose}")
            raise Error("Unable to move to pose")

    def _call_move_cartesian_service(self, x, y, z, v_max, acc):
        """
        Calls the move_cartesian callback
        """
        self.get_logger().info(f"Moving {x},{y},{z}")

        request = MoveCartesian.Request()

        request.x = x
        request.y = y
        request.z = z
        request.max_velocity = v_max
        request.acceleration = acc

        future = self.create_client(MoveCartesian, "move_cartesian").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=10)

        if not future.done():
            raise Error("Timeout reached when calling move_cartesian service")

        result: MoveCartesian.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to move {x},{y},{z}")
            raise Error("Unable to move to location")

    def _call_pick_up_gear_service(self, x, y, z, object_width):
        """
        Calls the pick_up_gear callback
        """
        self.get_logger().info(f"Picking up gear")

        request = PickUpGear.Request()

        request.x = x
        request.y = y
        request.z = z
        request.object_width = object_width

        future = self.pick_up_gear_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=20)

        if not future.done():
            raise Error("Timeout reached when calling pick_up gear service")

        result: PickUpGear.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to pick up gear")
            raise Error("Unable to pick up gear")

    def _call_move_to_conveyor_service(self, x, y, z):
        """
        Calls the move_to_conveyor callback
        """
        self.get_logger().info(f"Placing gear on conveyor belt")

        request = MoveToConveyor.Request()

        request.x = x
        request.y = y
        request.z = z

        future = self.move_to_conveyor_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=20)

        if not future.done():
            raise Error("Timeout reached when calling move_to_conveyor service")

        result: MoveToConveyor.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to move_to_conveyor")
            raise Error("Unable to move to conveyor")

    def _call_move_to_position_service(self, p: Point, rot: float = 0.0):
        """
        Calls the move_to_position callback
        """
        self.get_logger().info(f"Moving to position ({p.x}, {p.y}, {p.z})")

        request = MoveToPosition.Request()
        request.target_position = p
        request.gripper_rotation = rot

        future = self.move_to_position_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=8)

        if not future.done():
            raise Error("Timeout reached when calling move_to_position service")

        result: MoveToPosition.Response
        result = future.result()

        if not result.success:
            raise Error(f"Unable to move to desired position [{p.x}, {p.y}, {p.z}]")

    def _calculate_world_pose(self, frame_id: str, rel_pose: Pose) -> Pose:
        """
        Looks up transform from the world to the given frame
        """
        try:
            t = self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {frame_id} to world: {ex}")
            raise Error("Unable to transform between frames")

        return multiply_pose(convert_transform_to_pose(t), rel_pose)
