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
    PutGearDown,
)

from conveyor_interfaces.srv import EnableConveyor, SetConveyorState
from gear_place.transform_utils import multiply_pose, convert_transform_to_pose

from geometry_msgs.msg import Pose, Point

from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth

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
        self.put_gear_down_client = self.create_client(PutGearDown, "put_gear_down")

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

        future = self.create_client(PickUpGear, "pick_up_gear").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if not future.done():
            raise Error("Timeout reached when calling pick_up_gear service")

        result: PickUpGear.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to pick up gear")
            raise Error("Unable to pick up gear")

    def _call_put_gear_down_service(self, z):
        """
        Calls the put_gear_down callback
        """
        self.get_logger().info(f"Putting gear down")

        request = PutGearDown.Request()
        x_center = 320
        y_center = 240
        c=0
        object_depth = ObjectDepth((x_center,y_center))
        rclpy.spin_once(object_depth)
        while object_depth.dist_z in [0,None]:
            if c%2:
                x_center+=1
            else:
                y_center+=1
            c+=1
            object_depth.destroy_node()
            object_depth = ObjectDepth((x_center,y_center))
            rclpy.spin_once(object_depth)
        request.z = object_depth.dist_z *-1 +0.1

        future = self.create_client(PutGearDown, "put_gear_down").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if not future.done():
            raise Error("Timeout reached when calling put gear down service")

        result: PutGearDown.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to put gear down")
            raise Error("Unable to put gear down")

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


class ConveyorClass(Node):
    def _init__(self):
        # Service Clients
        self.enable_conveyor_client = self.create_client(
            EnableConveyor, "enable_conveyor"
        )
        self.set_conveyor_state_client = self.create_client(
            SetConveyorState, "set_conveyor_state"
        )

    def _enable_conveyor_service(self, enable: bool):
        """
        Calls the enable_conveyor callback
        """
        self.get_logger().info(
            ("Enabling" if enable else "Disabling"), "the conveyor belt"
        )

        request = EnableConveyor.Request()
        request.enable = enable

        future = self.enable_conveyor_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=8)

        if not future.done():
            raise Error("Timeout reached when calling enable_conveyor service")

        result: EnableConveyor.Response
        result = future.result()

        if not result.success:
            raise Error(f"Unable to enable the conveyor belt")

    def _set_conveyor_state_service(self, speed: float, direction: float):
        """
        Calls the set_conveyor_state callback
        """
        self.get_logger().info(
            "Moving the conveyor",
            ("forward" if direction == 0 else "backward"),
            f"at a speed of {speed}",
        )
        request = SetConveyorState.Request()
        request.speed = speed
        request.direction = direction
        
        future = self.set_conveyor_state_client.call_async(request)
        
        rclpy.spin_until_future_complete(self,future,timeout_sec=20)
        
        if not future.done():
            raise Error("Tiemout reached when calling set_conveyor_state service")
        
        result: SetConveyorState.Response
        result = future.result()
        
        if not result.success:
            raise Error(f"Unable to move the conveyor belt")
