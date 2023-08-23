import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from time import sleep

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from gear_place_interfaces.srv import (
    MoveCartesian,
    MoveToNamedPose,
    PickUpGear,
    MoveToPosition,
    PutGearDown,
    PickUpMovingGear,
)

from conveyor_interfaces.srv import EnableConveyor, SetConveyorState
from gear_place.transform_utils import multiply_pose, convert_transform_to_pose

from geometry_msgs.msg import Pose, Point

from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from gear_place.moving_gear import MovingGear


class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return repr(self.value)


def norm(x: float, y: float, z: float):
    return __import__("math").sqrt(x**2 + y**2 + z**2)


class GearPlace(Node):
    def _init__(self):
        # super().__init__("gear_place")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_transforms = []
        # Service Clients
        self.move_to_named_pose_client = self.create_client(
            MoveToNamedPose, "move_to_named_pose"
        )
        self.move_cartesian_client = self.create_client(MoveCartesian, "move_cartesian")
        self.pick_up_gear_client = self.create_client(PickUpGear, "pick_up_gear")
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

    def _call_pick_up_gear_service(self, object_width):
        """
        Calls the pick_up_gear callback
        """
        self.x_offset = 0.039  # offset from the camera to the gripper
        self.y_offset = 0.03  # offset from the camera to the gripper
        z_movement = (
            -0.247
        )  # z distance from the home position to where the gripper can grab the gear
        self.get_logger().info(f"Picking up gear")
        gear_center_target = [0 for i in range(3)]
        while (
            gear_center_target.count(0) == 3 or None in gear_center_target
        ):  # runs until valid coordinates are found
            find_object = FindObject()
            rclpy.spin_once(find_object)  # Finds the gear
            c = 0
            while (
                find_object.ret_cent_gear().count(None) != 0
            ):  # Runs and guarantees that none of the coordinates are none type
                c += 1

                if c % 5 == 0:
                    self._call_move_cartesian_service(
                        0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
                    )  # Moves to the center of the cart
                    sleep(1)
                else:
                    find_object.destroy_node()
                    find_object = FindObject()
                    rclpy.spin_once(find_object)
            object_depth = ObjectDepth(find_object.ret_cent_gear())
            rclpy.spin_once(object_depth)  # Gets the distance from the camera
            object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
            find_object.destroy_node()
            gear_center_target[0] = object_depth.dist_x
            gear_center_target[1] = object_depth.dist_y
            gear_center_target[2] = object_depth.dist_z
            sleep(0.2)  # sleeps between tries
        print(gear_center_target)

        request = PickUpGear.Request()

        request.x = -1 * object_depth.dist_y + self.x_offset
        request.y = -1 * object_depth.dist_x + self.y_offset
        request.z = z_movement
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

    def _call_put_gear_down_service(self):
        """
        Calls the put_gear_down callback
        """
        self.get_logger().info(f"Putting gear down")

        request = PutGearDown.Request()
        # x_point = 450
        # y_point = 300
        # object_depth = ObjectDepth((x_point, y_point))
        # rclpy.spin_once(object_depth)
        # c=0
        # while object_depth.dist_z in [0, None]:
        #     c+=1
        #     if c%15==0:
        #         if c%2==0: x_point+=1
        #         else: y_point+=1
        #     object_depth.destroy_node()
        #     object_depth = ObjectDepth((x_point, y_point))
        #     rclpy.spin_once(object_depth)
        #     print("Z_value:", object_depth.dist_z)
        #     sleep(0.2)
        # request.z = object_depth.dist_z * -1 + 0.07
        z = -0.247
        request.z = z + 0.0005
        future = self.create_client(PutGearDown, "put_gear_down").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if not future.done():
            raise Error("Timeout reached when calling put gear down service")

        result: PutGearDown.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to put gear down")
            raise Error("Unable to put gear down")

    def _call_pick_up_moving_gear_service(self, object_width):
        """
        Calls the pick_up_moving_gear callback
        """
        moving_gear = MovingGear()
        c=0
        while not moving_gear.found_gear:
            moving_gear.run()
            if not moving_gear.found_gear:
                c+=1
                self._call_move_cartesian_service(
                        0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
                )  # Moves to the center of the cart
                sleep(1)
        self.x_offset = 0.039  # offset from the camera to the gripper
        self.y_offset = 0.03  # offset from the camera to the gripper
        z_movement = -0.2465
        velocity = 0.15
        acceleration = 0.2
        pick_up_constant = velocity/acceleration+abs(z_movement)/velocity+2 #time that it takes for the fr3 to open gripper,move down, and grasp the gear
        slope, intercept = moving_gear.distance_formula()

        intersection_time = (
            -(velocity**2) / acceleration - velocity * pick_up_constant - intercept
        ) / (slope - velocity)
        distance_at_intersection = moving_gear.distance_to_point(
            moving_gear.point_from_time(intersection_time)
        )

        if (velocity**2) / acceleration > distance_at_intersection:
            velocity = distance_at_intersection / (velocity / acceleration) * 0.9
            intersection_time = (
                -(velocity**2) / acceleration
                - velocity * pick_up_constant
                - intercept
            ) / (slope - velocity)
        request = PickUpMovingGear.Request()
        x_value, y_value = moving_gear.point_from_time(intersection_time)
        request.x = y_value * -1 + self.x_offset 
        request.y = x_value * -1 + self.y_offset
        request.z = z_movement
        request.object_width = object_width

        future = self.create_client(PickUpMovingGear, "pick_up_moving_gear").call_async(
            request
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if not future.done():
            raise Error("Timeout reached when calling pick_up_moving_gear service")

        result: PickUpMovingGear.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to pick up gear")
            raise Error("Unable to pick up gear")

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

        rclpy.spin_until_future_complete(self, future, timeout_sec=20)

        if not future.done():
            raise Error("Tiemout reached when calling set_conveyor_state service")

        result: SetConveyorState.Response
        result = future.result()

        if not result.success:
            raise Error(f"Unable to move the conveyor belt")
