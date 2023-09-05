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
    OpenGripper,
)

from conveyor_interfaces.srv import EnableConveyor, SetConveyorState
from gear_place.transform_utils import multiply_pose, convert_transform_to_pose

from geometry_msgs.msg import Pose, Point

from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from gear_place.moving_gear import MovingGear
from gear_place.multiple_gears import MultipleGears
from math import sqrt


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
        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self, timeout_sec=0)
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
        self.x_offset = 0.0425  # offset from the camera to the gripper
        self.y_offset = 0.03  # offset from the camera to the gripper
        z_movement = (
            -0.247
        )  # z distance from the home position to where the gripper can grab the gear
        self.get_logger().info(f"Picking up gear")
        gear_center_target = [0 for _ in range(3)]
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
                    # self._call_move_cartesian_service(
                    #     0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
                    # )  # Moves to the center of the cart
                    sleep(1)
                else:
                    find_object.destroy_node()
                    find_object = FindObject()
                    rclpy.spin_once(find_object)
            object_depth = ObjectDepth([find_object.ret_cent_gear()])
            rclpy.spin_once(object_depth)  # Gets the distance from the camera
            object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
            find_object.destroy_node()
            gear_center_target = object_depth.coordinates[0]
            # sleep(0.2)  # sleeps between tries
        print(gear_center_target)

        request = PickUpGear.Request()

        request.x = -1 * gear_center_target[1] + self.x_offset
        request.y = -1 * gear_center_target[2] + self.y_offset
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

    def _call_put_gear_down_service(self, z):
        """
        Calls the put_gear_down callback
        """
        self.get_logger().info(f"Putting gear down")

        request = PutGearDown.Request()
        z_movement = max(
            -0.247, z + 0.0435
        )  # z distance from current position to the gear
        request.z = z_movement + 0.0005
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
        c = 0
        while not moving_gear.found_gear or len(moving_gear.x_vals) == 0:
            moving_gear.run()
            if not moving_gear.found_gear:
                c += 1
                self._call_move_cartesian_service(
                    0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
                )  # Moves to the center of the cart
                sleep(1)
        self.x_offset = 0.042  # offset from the camera to the gripper
        self.y_offset = 0.03  # offset from the camera to the gripper
        z_movement = -0.2465
        velocity = 0.15
        acceleration = 0.2
        pick_up_constant = (
            velocity / acceleration + abs(z_movement) / velocity + 0.1
        )  # time that it takes for the fr3 to open gripper,move down, and grasp the gear
        slope, intercept = moving_gear.distance_formula()

        intersection_time = (
            -(velocity**2) / acceleration - velocity * pick_up_constant - intercept
        ) / (
            slope - velocity
        )  # calculates the time where the pick up time equals the gear position
        distance_at_intersection = moving_gear.distance_to_point(
            moving_gear.point_from_time(intersection_time)
        )  # Calculates the distance of the gear to the camera at intersection time

        if (
            velocity**2
        ) / acceleration > distance_at_intersection:  # runs if max velocity needs to be lowered in move_cartesian
            velocity = (
                distance_at_intersection / (velocity / acceleration) * 0.9
            )  # lowers the velocity
            intersection_time = (
                -(velocity**2) / acceleration
                - velocity * pick_up_constant
                - intercept
            ) / (
                slope - velocity
            )  # calculates the new intersection time
        request = PickUpMovingGear.Request()
        if (
            abs(moving_gear.x_pix[1] - moving_gear.x_pix[0]) > 2
            or abs(moving_gear.y_pix[1] - moving_gear.y_pix[0]) > 2
        ):  # runs if the gear is moving
            x_value, y_value = moving_gear.point_from_time(intersection_time)
            request.x = y_value * -1 + self.x_offset
            request.y = x_value * -1 + self.y_offset
        else:  # runs if the gear is stationary
            print("Gear not moving")
            request.x = moving_gear.y_vals[0] * -1 + self.x_offset
            request.y = moving_gear.x_vals[0] * -1 + self.y_offset
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

    def remove_identical_points(self, arr):
        """
        Removes duplicate coordinates from different positions
        """
        bad_measurements = []
        for i in range(len(arr) - 1):
            for j in range(i + 1, len(arr)):
                if (
                    sqrt((arr[i][0] - arr[j][0]) ** 2 + (arr[i][1] - arr[j][1]) ** 2)
                    <= 0.03
                ):  # Gets rid of the points which are within 30mm of each other
                    bad_measurements.append(
                        j
                    )  # ensures that the first instance of a valid gear is saved
        bad_measurements = list(set(bad_measurements))  # removes duplicated indicies
        print(len(arr))
        print(bad_measurements)
        bad_measurements = sorted(bad_measurements)[
            ::-1
        ]  # sorts the indicies in decending order so the correct values are removed in next loop
        for ind in bad_measurements:  # deletes duplicated gears
            del arr[ind]
        return arr

    def find_distance(self, arr):
        return sqrt(arr[0] ** 2 + arr[1] ** 2)

    def _call_pick_up_multiple_gears(self, object_width):
        """
        Scans the area for gears. Finds the distances between the center of each gear and the home position and picks up each gear.
        """
        distances_from_home = []
        robot_moves = [
            [0.0, -0.1],
            [-0.1, 0.0],
            [-0.1, 0.0],
            [-0.1, 0.0],
            [0.0, 0.1],
            [0.1, 0.0],
            [0.1, 0.0],
            [0.1, 0.1],
            [-0.1, 0.0],
            [-0.1, 0.0],
            [-0.1, 0.0],
        ]  # cartesian movements starting at home position. Scans the area in front of the robot.
        x_movements = [a[0] for a in robot_moves]  # just the x direction movements
        y_movements = [a[1] for a in robot_moves]  # just the y direction movements
        self.get_logger().info(f"Scanning for gears")

        for ind in range(len(robot_moves)+1):  # loops through the scanning positions
            c = 0
            gear_center_target = [[0 for _ in range(3)]]
            while (
                (
                    [0, 0, 0] in gear_center_target
                    or sum([cent.count(None) for cent in gear_center_target]) > 0
                )
                and len(gear_center_target) > 0
                and c < 3
            ):  # runs until nothing is found, while something is found but coordinates are not, or if it runs 5 times with no results
                c += 1
                gear_center_target = []  # holds the coordinates for the gear centers
                multiple_gears = MultipleGears()
                rclpy.spin_once(
                    multiple_gears
                )  # finds multiple gears if there are multiple
                while (
                    sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
                    or not multiple_gears.ran
                ):  # loops until it has run and until there are no None values
                    multiple_gears.destroy_node()
                    multiple_gears = MultipleGears()
                    rclpy.spin_once(multiple_gears)
                object_depth = ObjectDepth(multiple_gears.g_centers)
                rclpy.spin_once(object_depth)  # Gets the distance from the camera
                object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
                for coord in object_depth.coordinates:
                    gear_center_target.append(coord)
                    if (
                        coord.count(0)!=3
                    ):  # adds coordinates if not all 0. Duplicates are removed later
                        distances_from_home.append(
                            (
                                -1 * coord[1] + sum(x_movements[:ind]),
                                -1 * coord[0] + sum(y_movements[:ind]),
                                -1 * coord[2],
                            )
                        )
                multiple_gears.destroy_node()

            for (
                arr
            ) in gear_center_target:  # adds the points to list which holds all points
                if arr != [0, 0, 0]:
                    distances_from_home.append(
                        (
                            -1 * arr[1] + sum(x_movements[:ind]),
                            -1 * arr[0] + sum(y_movements[:ind]),
                            -1 * arr[2],
                        )
                    )  # adds the previous movements so that the measurements are from the home position instead of the current position
            if ind != len(robot_moves):
                self._call_move_cartesian_service(
                    robot_moves[ind][0], robot_moves[ind][1], 0.0, 0.15, 0.2
                )  # moves to the next position

        distances_from_home = self.remove_identical_points(
            distances_from_home
        )  # since gears will be repeated from different positions, repetitions are removed

        distances_from_home = [
            distances_from_home[i]
            for i in range(len(distances_from_home))
            if self.find_distance(distances_from_home[i]) <= 0.27
        ]  # removes points which are too far from the home position

        self.get_logger().info(
            f"{len(distances_from_home)} gears found. Picking up the gears"
        )  # outputs the number of gears found
        for movment in distances_from_home:
            self.get_logger().info("Movement: " + str(movment))

        self._call_move_to_named_pose_service("home")
        last_point = [0, 0]
        offset_needed = True
        for (
            gear_point
        ) in distances_from_home:  # loops through the movements to the gears
            move = [
                gear_point[i] - last_point[i] for i in range(2)
            ]  # finds the next movement to the next gear
            last_point = gear_point
            self.get_logger().info("Next_move:" + str(move))
            self._call_open_gripper_service()  # opens the gripper
            self._call_pick_up_gear_coord_service(
                offset_needed, move[0], move[1], gear_point[2], object_width
            )  # picks up the gear
            self._call_put_gear_down_camera(gear_point[2])  # puts the gear down
            offset_needed = False

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

    def _call_open_gripper_service(self):
        """
        Calls the move_to_position callback
        """
        self.get_logger().info("Opening gripper")

        request = OpenGripper.Request()

        future = self.create_client(OpenGripper, "open_gripper").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=8)

        if not future.done():
            raise Error("Timeout reached when calling move_to_position service")

        result: OpenGripper.Response
        result = future.result()

        if not result.success:
            raise Error("Unable to move to open gripper")

    def _call_pick_up_gear_coord_service(self, offset_bool, x, y, z, object_width):
        """
        Calls the pick_up_gear callback
        """
        self.x_offset = 0.03975  # offset from the camera to the gripper
        self.y_offset = 0.03  # offset from the camera to the gripper
        z_movement = max(
            -0.247, z + 0.0435
        )  # z distance from the home position to where the gripper can grab the gear
        self.get_logger().info(f"Picking up gear")
        request = PickUpGear.Request()

        request.x = x + (self.x_offset if offset_bool else 0)
        request.y = y + (self.y_offset if offset_bool else 0)
        
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

    def _call_put_gear_down_camera(self, z):
        """
        Uses the camera to put down the gear at the correct height
        """
        self.get_logger().info(f"Putting gear down")

        run_bool = False
        depth_vals = []
        x_center = 100
        y_center = 100
        while len(depth_vals) < 5:
            self.get_logger().info(
            "Depth Values:" + ", ".join([str(val) for val in depth_vals])
            )
            camera_points = [
                (x_center - 1 + i, y_center - 1 + j) for i in range(3) for j in range(3)
            ]
            if run_bool:
                object_depth.destroy_node()
            object_depth = ObjectDepth(camera_points)
            rclpy.spin_once(object_depth)
            run_bool = True
            for coord in object_depth.coordinates:
                if coord[2] not in [None, 0] and coord[2]<0.7:
                    depth_vals.append(coord[2])
            x_center += 3
            y_center += 3

        self.get_logger().info(
            "Depth Values:" + ", ".join([str(val) for val in depth_vals])
        )

        request = PutGearDown.Request()
        request.z = max(-1 * (sum(depth_vals) / len(depth_vals)) + 0.085,max(
            -0.247, z + 0.0435
        ))
        future = self.create_client(PutGearDown, "put_gear_down").call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30)

        if not future.done():
            raise Error(
                "Timeout reached when calling put gear down using camera service"
            )

        result: PutGearDown.Response
        result = future.result()

        if not result.success:
            self.get_logger().error(f"Unable to put gear down using camera")
            raise Error("Unable to put gear down using camera")

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
            ("Enabling " if enable else "Disabling ") + "the conveyor belt"
        )

        request = EnableConveyor.Request()
        request.enable = enable

        future = self.create_client(EnableConveyor, "enable_conveyor").call_async(
            request
        )

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

        future = self.create_client(SetConveyorState, "set_conveyor_state").call_async(
            request
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=20)

        if not future.done():
            raise Error("Tiemout reached when calling set_conveyor_state service")

        result: SetConveyorState.Response
        result = future.result()

        if not result.success:
            raise Error(f"Unable to move the conveyor belt")
