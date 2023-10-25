import rclpy
from rclpy.node import Node
from rclpy.time import Duration

import yaml

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
  PutDownForce,
  GetCameraAngle,
  MoveCartesianAngle,
  RotateSingleJoint,
  MoveToJointPosition,
  GetJointPositions,
  MoveCartesianSmooth
)

from conveyor_interfaces.srv import EnableConveyor, SetConveyorState
from gear_place.transform_utils import convert_transform_to_pose
from gear_place.gear_place_utilities import (
    norm, 
    avg, 
    dist_from_point, 
    distance_between_two_points, 
    rotate_points_around_angle,
    X_OFFSET,
    Y_OFFSET,
    Z_CAMERA_OFFSET,
    Z_TO_TABLE,
    convert_color_to_depth,
    convert_color_to_depth_radius,
    average_of_points,
    remove_identical_points,
    closest_to_center,
    find_distance
  )

from geometry_msgs.msg import Pose, Point

from gear_place.find_object_color import FindObjectColor
from gear_place.find_object import FindObject
from gear_place.object_depth import ObjectDepth
from gear_place.moving_gear import MovingGear
from gear_place.multiple_gears import MultipleGears, MultipleGearsColor
from math import sqrt, sin, cos, pi

class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)

class GearPlace(Node):
  def __init__(self):
      super().__init__('gear_place')

      # TF
      self.tf_buffer = Buffer()
      self.tf_listener = TransformListener(self.tf_buffer, self)

      self.tf_broadcaster = StaticTransformBroadcaster(self)
      self.static_transforms = []

      # Current angle
      self.current_camera_angle = 0.0

      # Current joint positions
      self.current_joint_positions = []

      # Camera connected
      self.connected = False

    # TODO
      # Camera to end effector transform
    #   cam_to_ee_tranform = self.tf_buffer.lookup_transform("fr3_hand","camera_mount", rclpy.time.Time())
    #   cam_to_ee_pose = convert_transform_to_pose(cam_to_ee_tranform)
    #   self.x_offset = cam_to_ee_pose.position.x
    #   self.y_offset = cam_to_ee_pose.position.y
        
      # Service Clients
      self.move_to_named_pose_client = self.create_client(MoveToNamedPose, "move_to_named_pose")
      self.move_cartesian_client = self.create_client(MoveCartesian, "move_cartesian")
      self.pick_up_gear_client = self.create_client(PickUpGear, "pick_up_gear")
      self.move_to_position_client = self.create_client(MoveToPosition, "move_to_position")
      self.put_gear_down_client = self.create_client(PutGearDown, "put_gear_down")
      self.pick_up_moving_gear_client = self.create_client(PickUpMovingGear, "pick_up_moving_gear")
      self.open_gripper_client = self.create_client(OpenGripper, "open_gripper")
      self.put_down_force_client = self.create_client(PutDownForce, "put_down_force")
      self.get_camera_angle_client = self.create_client(GetCameraAngle, "get_camera_angle")
      self.move_cartesian_angle_client = self.create_client(MoveCartesianAngle, "move_cartesian_angle")
      self.rotate_single_joint_client = self.create_client(RotateSingleJoint, "rotate_single_joint")
      self.move_to_joint_position_client = self.create_client(MoveToJointPosition, "move_to_joint_position")
      self.get_joint_positions_client = self.create_client(GetJointPositions, "get_joint_positions")
      self.move_cartesian_smooth_client = self.create_client(MoveCartesianSmooth, "move_cartesian_smooth")
      self.enable_conveyor_client = self.create_client(
          EnableConveyor, "conveyor/enable"
      )
      self.set_conveyor_state_client = self.create_client(
          SetConveyorState, "conveyor/set_state"
      )

  def wait(self, duration: float):
      start = self.get_clock().now()

      while self.get_clock().now() <= start + Duration(seconds=duration):
          try:
              rclpy.spin_once(self, timeout_sec=0)
          except KeyboardInterrupt:
              raise Error("Ctrl+C pressed")

  def call_open_gripper_service(self):
      """
      Calls the open_gripper callback
      """
      self.get_logger().info("Opening gripper")

      future = self.open_gripper_client.call_async(OpenGripper.Request())

      rclpy.spin_until_future_complete(self, future, timeout_sec=8)

      if not future.done():
          raise Error("Timeout reached when calling open_gripper service")

      result: OpenGripper.Response
      result = future.result()

      if not result.success:
          raise Error("Unable to move to open gripper")

  def calculate_world_pose(self, frame_id: str) -> Pose:
        # Lookup transform from world to frame_id
        try:
            t = self.tf_buffer.lookup_transform('world', frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {frame_id} to world: {ex}')
            raise Error("Unable to transform between frames")
        
        return convert_transform_to_pose(t)

  # ===========================================================
  #               cartesian movment functions
  # ===========================================================

  def call_move_cartesian_service(self, x : float, y : float, z : float, v_max : float, acc : float):
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

      future = self.move_cartesian_client.call_async(request)

      rclpy.spin_until_future_complete(self, future, timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when calling move_cartesian service")

      result: MoveCartesian.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to move {x},{y},{z}")
          raise Error("Unable to move to location")
    
  def call_move_cartesian_angle_service(self, x : float, y : float, z : float, v_max : float, acc : float, angle : float):
      """
      Calls the move_cartesian_angle callback
      """
      self.get_logger().info(f"Moving {x},{y},{z} at angle {angle}")

      request = MoveCartesianAngle.Request()

      request.x = x
      request.y = y
      request.z = z
      request.max_velocity = v_max
      request.acceleration = acc
      request.angle = angle

      future = self.move_cartesian_angle_client.call_async(request)

      rclpy.spin_until_future_complete(self, future, timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when calling move_cartesian_angle service")

      result: MoveCartesianAngle.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to move {x},{y},{z}")
          raise Error("Unable to move to location")
  
  def call_move_cartesian_smooth_service(self, x : float, y : float, z : float, v_max : float, acc : float):
      """
      Calls the move_cartesian_smooth callback
      """
      self.get_logger().info(f"Moving {x},{y},{z}")

      request = MoveCartesianSmooth.Request()
      request.x = x
      request.y = y
      request.z = z
      request.max_velocity = v_max
      request.acceleration = acc

      future = self.move_cartesian_smooth_client.call_async(request)

      rclpy.spin_until_future_complete(self, future, timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when calling move_cartesian_smooth service")

      result: MoveCartesianSmooth.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to move {x},{y},{z}")
          raise Error("Unable to move to location")
   
  # ===========================================================
  #                   scanning functions
  # ===========================================================

  def scan_multiple_gears_grid(self,robot_moves = [
          [0.0, -0.1],
          [-0.1, 0.0],
          [-0.1, 0.0],
          [0.0, 0.1],
          [0.1, 0.0],
          [0.1, 0.1],
          [-0.1, 0.0],
          [-0.1, 0.0]])->(list,dict):
      """
      Scans the area for gears. Finds the distances between the center of each gear and the home position and picks up each gear.
      """
      self.call_move_to_named_pose_service("home")
      distances_from_home = []
      x_movements = [a[0] for a in robot_moves]  # just the x direction movements
      y_movements = [a[1] for a in robot_moves]  # just the y direction movements
      self.get_logger().info(f"Scanning for gears")
      gears_found = 0
      updated_radius_vals = {}
      while gears_found == 0:
          for ind in range(len(robot_moves)+1):  # loops through the scanning positions
              self.call_get_joint_positions()
              self.get_logger().info("Current joint positions: " + ", ".join([str(val) for val in self.current_joint_positions]))
              self.call_get_camera_angle()
              self.get_logger().info(f"Current camera angle in radians: {self.current_camera_angle}")
              for _ in range(2):  # runs until nothing is found, while something is found but coordinates are not, or if it runs 5 times with no results
                  multiple_gears = MultipleGears(self.connected)
                  rclpy.spin_once(multiple_gears)  # finds multiple gears if there are multiple
                  self.connected = multiple_gears.connected
                  while (
                      sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
                      or not multiple_gears.ran
                  ):  # loops until it has run and until there are no None values
                      multiple_gears.destroy_node()
                      multiple_gears = MultipleGears(self.connected)
                      rclpy.spin_once(multiple_gears)
                      self.connected = multiple_gears.connected
                  object_depth = ObjectDepth(multiple_gears.g_centers, multiple_gears.dist_points)
                  rclpy.spin_once(object_depth)  # Gets the distance from the camera
                  object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
                  for coord in object_depth.coordinates:
                      if coord.count(0.0)==0:  # adds coordinates if not all 0. Duplicates are removed later
                          distances_from_home.append(
                              (
                                  -1 * coord[1] + sum(x_movements[:ind]),
                                  -1 * coord[0] + sum(y_movements[:ind]),
                                  -1 * coord[2],
                              )
                          )
                          updated_radius_vals[(
                                  -1 * coord[1] + sum(x_movements[:ind]),
                                  -1 * coord[0] + sum(y_movements[:ind]),
                                  -1 * coord[2],
                              )] = object_depth.radius_vals[coord]
                  multiple_gears.destroy_node()
              if ind != len(robot_moves):
                  self.call_move_cartesian_smooth_service(
                      robot_moves[ind][0], robot_moves[ind][1], 0.0, 0.15, 0.2
                  )  # moves to the next position

          distances_from_home = remove_identical_points(distances_from_home, updated_radius_vals)  # since gears will be repeated from different positions, repetitions are removed

          distances_from_home = [
              distances_from_home[i]
              for i in range(len(distances_from_home))
              if find_distance(distances_from_home[i]) <= 0.4
          ]  # removes points which are too far from the home position
          gears_found = len(distances_from_home)
          if gears_found==0:
              self.call_move_to_named_pose_service("home")
              self.get_logger().info("No gears found. Trying again")
      self.get_logger().info(
          f"{len(distances_from_home)} gears found."
      )  # outputs the number of gears found
      return distances_from_home,updated_radius_vals
  
  def single_scan_multiple_gears(self)->(list,dict):
    """
    Scans the area for gears. Uses only one position to find the gears.
    """
    self.call_move_to_named_pose_service("high_scan")
    self.call_get_camera_angle()
    distances_from_home = []
    self.get_logger().info(f"Scanning for gears")
    gears_found = 0
    updated_radius_vals = {}
    while gears_found == 0:
        self.call_get_camera_angle()
        self.get_logger().info(f"Current camera angle in radians: {self.current_camera_angle}")
        for _ in range(3):  # runs until nothing is found, while something is found but coordinates are not, or if it runs 5 times with no results
            multiple_gears_color = MultipleGearsColor(self.connected)
            rclpy.spin_once(multiple_gears_color)  # finds multiple gears if there are multiple
            self.connected = multiple_gears_color.connected
            while (
                sum([cent.count(None) for cent in multiple_gears_color.g_centers]) != 0
                or not multiple_gears_color.ran
            ):  # loops until it has run and until there are no None values
                multiple_gears_color.destroy_node()
                multiple_gears_color = MultipleGearsColor(self.connected)
                rclpy.spin_once(multiple_gears_color)
                self.connected = multiple_gears_color.connected
            object_depth = ObjectDepth([convert_color_to_depth(point) for point in multiple_gears_color.g_centers], 
                                       {convert_color_to_depth(point):[convert_color_to_depth(p) for p in multiple_gears_color.dist_points[point]] for point in multiple_gears_color.g_centers})
            rclpy.spin_once(object_depth)  # Gets the distance from the camera
            object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
            for coord in object_depth.coordinates:
                if coord.count(0.0)==0:  # adds coordinates if not all 0. Duplicates are removed later
                    distances_from_home.append(
                        (
                            -1 * coord[1],
                            -1 * coord[0],
                            -1 * coord[2]
                        )
                    )
                    updated_radius_vals[(
                            -1 * coord[1],
                            -1 * coord[0],
                            -1 * coord[2])] = object_depth.radius_vals[coord]
            multiple_gears_color.destroy_node()

        distances_from_home = remove_identical_points(distances_from_home, updated_radius_vals)  # since gears will be repeated from different positions, repetitions are removed

        distances_from_home = [
            distances_from_home[i]
            for i in range(len(distances_from_home))
            if find_distance(distances_from_home[i]) <= 0.7
        ]  # removes points which are too far from the home position
        gears_found = len(distances_from_home)
        if gears_found==0:
            self.call_move_to_named_pose_service("high_scan")
            self.get_logger().info("No gears found. Trying again")
    self.get_logger().info(
        f"{len(distances_from_home)} gears found"
    )  # outputs the number of gears found
    return distances_from_home, updated_radius_vals

  def select_scan(self, type_scan="single",robot_moves=[]):
      if type_scan=="single":
          return self.single_scan_multiple_gears()
      elif type_scan=="grid":
          return (self.scan_multiple_gears_grid() if robot_moves==[] else self.scan_multiple_gears_grid(robot_moves))
      else:
          self.get_logger().error(f"{type_scan} is not a valid scan type. Please choose between \"single\" and \"grid\"")
          return [],{}
  
  # ===========================================================
  #                 pick up gear functions
  # ===========================================================

  def call_pick_up_gear_service(self, object_width : float,depth_or_color:bool):
      """
      Calls the pick_up_gear callback
      """
      self.get_logger().info(f"Picking up gear")
      gear_center_target = [0 for _ in range(3)]
      requested_class = FindObject if depth_or_color else FindObjectColor
      while (
          gear_center_target.count(0) == 3 or None in gear_center_target
      ):  # runs until valid coordinates are found
          find_object = requested_class()
          rclpy.spin_once(find_object) # Finds the gear
          c = 0
          while (
              find_object.ret_cent_gear().count(None) != 0
          ):  # Runs and guarantees that none of the coordinates are none type
              c += 1

              if c % 5 == 0:
                  self.call_move_cartesian_smooth_service(
                      0.05, 0.05 * (-1 if c % 2 == 1 else 1), 0.0, 0.15, 0.2
                  )  # Moves to the center of the cart
                  sleep(1)
              else:
                  find_object.destroy_node()
                  find_object = requested_class()
                  rclpy.spin_once(find_object)
          object_depth = ObjectDepth([find_object.ret_cent_gear()] if depth_or_color else[convert_color_to_depth(find_object.ret_cent_gear())],{})
          rclpy.spin_once(object_depth)  # Gets the distance from the camera
          object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
          find_object.destroy_node()
          gear_center_target = object_depth.coordinates[0]
      self.get_logger().info("gear_center_target: "+str(gear_center_target))

      request = PickUpGear.Request()

      request.x = -1 * gear_center_target[1] + X_OFFSET
      request.y = -1 * gear_center_target[0] + Y_OFFSET
      request.z = -1 * gear_center_target[2] + Z_CAMERA_OFFSET + 0.0075
      request.object_width = object_width

      future = self.pick_up_gear_client.call_async(request)

      rclpy.spin_until_future_complete(self, future, timeout_sec=30)

      if not future.done():
          raise Error("Timeout reached when calling pick_up_gear service")

      result: PickUpGear.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to pick up gear")
          raise Error("Unable to pick up gear")

  def call_pick_up_gear_coord_service(self, offset_bool : bool, x : float, y : float, z : float, object_width : float, default_up : bool):
      """
      Calls the pick_up_gear callback
      """
      z_movement = max(
          Z_TO_TABLE, z + Z_CAMERA_OFFSET
      )  # z distance from the home position to where the gripper can grab the gear
      self.get_logger().info(f"Picking up gear")
      request = PickUpGear.Request()

      request.x = x + (X_OFFSET if offset_bool else 0)
      request.y = y + (Y_OFFSET if offset_bool else 0)
      
      request.z = z_movement
      request.object_width = object_width
      request.default_up = default_up

      future = self.pick_up_gear_client.call_async(request)

      rclpy.spin_until_future_complete(self, future, timeout_sec=30)

      if not future.done():
          raise Error("Timeout reached when calling pick_up_gear service")

      result: PickUpGear.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to pick up gear")
          raise Error("Unable to pick up gear")

  def pick_up_multiple_gears(self, distances_from_home: list, updated_radius_vals: list, object_width : float, starting_position: str, colors: list,depth_or_color: bool, put_down_type = "force", force = 0.1):
      for movment in distances_from_home:
          self.get_logger().info("Movement: " + str(movment))

      if starting_position != "current":
        self.call_move_to_named_pose_service(starting_position)
        if starting_position == "high_scan":
          self.call_move_cartesian_smooth_service(0.0,0.0,-0.16,0.15,0.2)
      last_point = [0, 0]
      offset_needed = True
      low_gear_threshold = 0.0275
      high_gear_thershold = 0.041
      if depth_or_color:
        for gear_point in distances_from_home:  # loops through the movements to the gears
            if updated_radius_vals[gear_point]!=0:
              thresholds = sorted([low_gear_threshold, high_gear_thershold,updated_radius_vals[gear_point]])
              gear_color = ["yellow", "orange", "green"][thresholds.index(updated_radius_vals[gear_point])]
              if gear_color in colors:
                move = [
                    gear_point[i] - last_point[i] for i in range(2)
                ]  # finds the next movement to the next gear
                last_point = gear_point
                self.get_logger().info("Next_move:" + str(move))
                self.get_logger().info(f"Picking up a {gear_color} gear with radius size of {updated_radius_vals[gear_point]}")
                self.call_open_gripper_service()  # opens the gripper
                
                if offset_needed:
                  self.call_move_cartesian_smooth_service(
                      move[0]+X_OFFSET, move[1]+Y_OFFSET, 0.0, 0.15, 0.2
                  )  # moves above the gear
                else:
                  self.call_move_cartesian_smooth_service(
                      move[0], move[1], 0.0, 0.15,0.2
                  )  # moves above the gear
                correct_gear = [0.0,0.0,0.0]
                counter = 0
                while (correct_gear in [[0.0,0.0,0.0],[None for _ in range(3)]] or sum(correct_gear)==0.0) and counter <3:
                  counter+=1
                  multiple_gears = MultipleGears(self.connected)
                  rclpy.spin_once(multiple_gears)  # finds multiple gears if there are multiple
                  self.connected = multiple_gears.connected
                  while (
                      sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
                      or not multiple_gears.ran
                  ):  # loops until it has run and until there are no None values
                      multiple_gears.destroy_node()
                      multiple_gears = MultipleGears(self.connected)
                      rclpy.spin_once(multiple_gears)
                      self.connected = multiple_gears.connected
                  object_depth = ObjectDepth(multiple_gears.g_centers, multiple_gears.dist_points)
                  rclpy.spin_once(object_depth)  # Gets the distance from the camera
                  multiple_gears.destroy_node()
                  object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
                  closest_gears = [coord 
                                   for coord in object_depth.coordinates 
                                   if gear_color == 
                                   ["yellow", "orange", "green"][sorted([low_gear_threshold, high_gear_thershold,object_depth.radius_vals[coord]]).index(object_depth.radius_vals[coord])]] #ensures that the gear color mateches
                  correct_gear_index = closest_to_center(closest_gears)
                  correct_gear = closest_gears[closest_to_center(closest_gears)] if correct_gear_index>0 else [0 for _ in range(3)]
                self.get_logger().info(", ".join([str(val) for val in correct_gear]))
                if correct_gear.count(0.0)>=1 or correct_gear.count(None)>=1:
                    self.get_logger().error("Second check above gear did not work. Attempting to pick up with current position")
                    self.call_pick_up_gear_coord_service(False,0.0,0.0, gear_point[2] - (0.01 if gear_color=="green" else 0), object_width, False)
                    z = gear_point[2] - (0.01 if gear_color=="green" else 0)
                else:
                  self.call_pick_up_gear_coord_service(
                      True, -1*correct_gear[1], -1*correct_gear[0],-1*correct_gear[2] - (0.01 if gear_color=="green" else 0), object_width, False
                  )
                  last_point=(last_point[0]+-1*correct_gear[1] +X_OFFSET,last_point[1]+-1*correct_gear[0]+Y_OFFSET)
                  z = -1*correct_gear[2] - (0.01 if gear_color=="green" else 0)
                self.call_get_joint_positions()
                self.put_gear_down_choose_type(put_down_type,z=z,force=force)
                self.call_move_to_joint_position(self.current_joint_positions)
                offset_needed = False
      else:
          low_gear_threshold = 0.03
          high_gear_thershold = 0.045
          for gear_point in distances_from_home:  # loops through the movements to the gears
              if updated_radius_vals[gear_point]!=0:
                thresholds = sorted([low_gear_threshold, high_gear_thershold,updated_radius_vals[gear_point]])
                gear_color = ["yellow", "orange", "green"][thresholds.index(updated_radius_vals[gear_point])]
                if gear_color in colors:
                    move = [
                        gear_point[i] - last_point[i] for i in range(2)
                    ]  # finds the next movement to the next gear
                    last_point = gear_point
                    self.get_logger().info("Next_move:" + str(move))
                    if updated_radius_vals[gear_point] ==0:
                        gear_color == "not found"
                        self.get_logger().info("Could not find gear color")
                    else:
                        thresholds = sorted([low_gear_threshold, high_gear_thershold,updated_radius_vals[gear_point]])
                        gear_color = ["yellow", "orange", "green"][thresholds.index(updated_radius_vals[gear_point])]
                        self.get_logger().info(f"Picking up a {gear_color} gear with radius size of {updated_radius_vals[gear_point]}")
                    self.call_open_gripper_service()  # opens the gripper
                    
                    if offset_needed:
                        self.call_move_cartesian_smooth_service(
                            move[0]+X_OFFSET, move[1]+Y_OFFSET, 0.0, 0.15, 0.2
                        )  # moves above the gear
                    else:
                        self.call_move_cartesian_smooth_service(
                            move[0], move[1], 0.0, 0.15,0.2
                        )  # moves above the gear
                    correct_gear = [0.0,0.0,0.0]
                    counter = 0
                    multiple_gears = MultipleGearsColor(self.connected)
                    rclpy.spin_once(multiple_gears)  # finds multiple gears if there are multiple
                    self.connected = multiple_gears.connected
                    while (
                        sum([cent.count(None) for cent in multiple_gears.g_centers]) != 0
                        or not multiple_gears.ran
                    ):  # loops until it has run and until there are no None values
                        multiple_gears.destroy_node()
                        multiple_gears = MultipleGearsColor(self.connected)
                        rclpy.spin_once(multiple_gears)
                        self.connected = multiple_gears.connected
                    while (correct_gear in [[0.0,0.0,0.0],[None for _ in range(3)]] or sum(correct_gear)==0.0) and counter <15:
                        counter+=1
                        object_depth = ObjectDepth([convert_color_to_depth(point) for point in multiple_gears.g_centers],
                                                {convert_color_to_depth(point):[convert_color_to_depth(p) for p in multiple_gears.dist_points[point]] for point in multiple_gears.g_centers})
                        rclpy.spin_once(object_depth)  # Gets the distance from the camera
                        object_depth.destroy_node()  # Destroys the node to avoid errors on next loop
                        closest_gears = [coord 
                                        for coord in object_depth.coordinates 
                                        if gear_color == 
                                        ["yellow", "orange", "green"][sorted([low_gear_threshold, high_gear_thershold,object_depth.radius_vals[coord]]).index(object_depth.radius_vals[coord])]] #ensures that the gear color mateches
                        correct_gear_index = closest_to_center(closest_gears)
                        correct_gear = closest_gears[closest_to_center(closest_gears)] if correct_gear_index>0 else [0 for _ in range(3)]
                    multiple_gears.destroy_node()
                    self.get_logger().info(", ".join([str(val) for val in correct_gear]))
                    if correct_gear.count(0.0)>=1 or correct_gear.count(None)>=1:
                        self.get_logger().error("Second check above gear did not work. Attempting to pick up with current position")
                        self.call_pick_up_gear_coord_service(False,0.005,0.0, gear_point[2], object_width,True)
                        last_point=(last_point[0]+0.005,last_point[1])
                    else:
                        self.call_pick_up_gear_coord_service(
                            True, -1*correct_gear[1], -1*correct_gear[0],-1*correct_gear[2], object_width, True
                        )
                        last_point=(last_point[0]+-1*correct_gear[1]+X_OFFSET,last_point[1]+-1*correct_gear[0]+Y_OFFSET)
                    #   self.call_put_gear_down_camera(-1*coorect_gear[2])  # puts the gear down
                    self.call_get_joint_positions()
                    self.put_gear_down_choose_type("force",force=0.1)
                    self.call_move_to_joint_position(self.current_joint_positions)
                    offset_needed = False

  # ===========================================================
  #                 put down gear functions
  # ===========================================================

  def call_put_gear_down_service(self, z : float):
    """
    Calls the put_gear_down callback
    """
    self.get_logger().info(f"Putting gear down")

    request = PutGearDown.Request()
    z_movement = max(
        Z_TO_TABLE, z + Z_CAMERA_OFFSET
    )  + 0.0005 # z distance from current position to the gear and makes sure it does not try to go below the table
    request.z = z_movement 
    future = self.put_gear_down_client.call_async(request)

    rclpy.spin_until_future_complete(self, future, timeout_sec=30)

    if not future.done():
        raise Error("Timeout reached when calling put gear down service")

    result: PutGearDown.Response
    result = future.result()

    if not result.success:
        self.get_logger().error(f"Unable to put gear down")
        raise Error("Unable to put gear down")

  def call_put_gear_down_camera(self, z : float):
      """
      Uses the camera to put down the gear at the correct height
      """
      self.get_logger().info(f"Putting gear down")
      depth_vals = []
      x_center = 100
      y_center = 100
      c=0
      while len(depth_vals) < 5:
          c+=1
          self.get_logger().info(
          "Depth Values: " + ", ".join([str(val) for val in depth_vals])
          )
          camera_points = [
              (x_center - 1 + i, y_center - 1 + j) for i in range(3) for j in range(3)
          ]
          if "object_depth" in locals():
              object_depth.destroy_node()
          object_depth = ObjectDepth(camera_points,{})
          rclpy.spin_once(object_depth)
          depth_vals += [coord[2] for coord in object_depth.coordinates if coord[2] not in [None, 0] and coord[2]<0.7]
          x_center += 3
          y_center += 3
          if c>=10 and len(depth_vals)<7: # fills the list with bad values if the loop runs for too long
              depth_vals = [100 for _ in range(10)]

      request = PutGearDown.Request()
      request.z = max(-1 * (avg(depth_vals)) + 0.0795,Z_TO_TABLE, z + Z_CAMERA_OFFSET) # does not go further down than where it picked it up
      future = self.put_gear_down_client.call_async(request)

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
    
  def call_put_down_force(self, force:float):
    """
    Calls the put_down_force callback
    """
    self.get_logger().info("Putting the gear down")

    request = PutDownForce.Request()
    
    request.force = force

    future = self.put_down_force_client.call_async(request)

    rclpy.spin_until_future_complete(self, future, timeout_sec=100)

    if not future.done():
        raise Error("Timeout reached when calling put_down_force service")

    result: PutDownForce.Response
    result = future.result()

    if not result.success:
        raise Error("Unable to put down gear using force motion generator")
    
  def put_gear_down_choose_type(self, put_down_type : str, z=0.0,force = 0.0):
      self.call_get_joint_positions()
      if put_down_type not in ["force", "camera", "value"] or (z == 0.0 and put_down_type!="force") or (put_down_type=="force" and force<=0.0):
          if put_down_type not in ["force", "camera", "value"]:
              self.get_logger().error(f"{put_down_type} is not a valid put down type. The valid options are [force, camera, value].")
          elif z == 0.0 and put_down_type!="force":
              self.get_logger().error(f"Both camera and value put down methods need non-zero z values.")
          else:
              self.get_logger().error(f"The force method needs a non-zero positive force value.")
          self.get_logger().info("Putting gear down using force method with force of 0.1.")
          self.call_put_down_force(0.1)
      self.call_put_gear_down_service(z) if put_down_type=="value" else (self.call_put_gear_down_camera(z) if put_down_type=="camera" else self.call_put_down_force(force))
      self.call_move_to_joint_position(self.current_joint_positions)

  # ===========================================================
  #                   moving gear functions
  # ===========================================================

  def call_move_above_gear(self):
      """
      Moves the robot above the gear
      """
      self.call_get_camera_angle()
      moving_gear = MovingGear()
      while not moving_gear.found_gear or len(moving_gear.x_vals) == 0:
          moving_gear.run()
      velocity = 0.15
      acceleration = 0.2
      slope, intercept = moving_gear.distance_formula()

      intersection_time = (
              -(velocity**2) / acceleration - intercept
          ) / (
              slope - velocity
          )
      distance_at_intersection = moving_gear.distance_to_point(
          moving_gear.point_from_time(intersection_time)
      )

      if (
          velocity**2
      ) / acceleration > distance_at_intersection:  # runs if max velocity needs to be lowered in move_cartesian
          velocity = (
              distance_at_intersection / (velocity / acceleration) * 0.9
          )  # lowers the velocity
          intersection_time = (
              -(velocity**2) / acceleration
              - intercept
          ) / (
              slope - velocity
          )
      
      x_value, y_value = moving_gear.point_from_time(intersection_time)
      
      ratio_x = x_value/(x_value+y_value)
      ratio_y = 1 - ratio_x
      
      final_x, final_y = moving_gear.point_from_time(intersection_time*3)
      gear_speed = (distance_between_two_points(moving_gear.x_vals, moving_gear.y_vals)) / (moving_gear.times[1]-moving_gear.times[0])

      second_acceleration = 0.3
      second_t1 = gear_speed/second_acceleration

      distance_lost_to_acc = gear_speed * second_t1 / 2
      
      self.call_move_cartesian_angle_service(y_value * -1 + X_OFFSET + ratio_y * distance_lost_to_acc,x_value * -1 + Y_OFFSET + ratio_x * distance_lost_to_acc,0.0,velocity, acceleration, self.current_camera_angle)
      self.call_move_cartesian_angle_service(final_y * -1 + X_OFFSET,final_x * -1 + Y_OFFSET, 0.0, gear_speed, second_acceleration,self.current_camera_angle)
  
  def call_pick_up_moving_gear_service(self, object_width : float):
      """
      Calls the pick_up_moving_gear callback
      """
      moving_gear = MovingGear()
      while not moving_gear.found_gear or len(moving_gear.x_vals) == 0:
          moving_gear.run()
      z_movement = -0.2465
      velocity = 0.15
      acceleration = 0.2
      pick_up_constant = (
          velocity / acceleration + abs(z_movement) / velocity + 0.55
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
          request.x = y_value * -1 + X_OFFSET
          request.y = x_value * -1 + Y_OFFSET
      else:  # runs if the gear is stationary
          self.get_logger().info("Gear not moving")
          request.x = moving_gear.y_vals[0] * -1 + X_OFFSET
          request.y = moving_gear.x_vals[0] * -1 + Y_OFFSET
      request.z = sum(moving_gear.z_height)/len(moving_gear.z_height) * -1 + Z_CAMERA_OFFSET
      request.object_width = object_width
      request.angle = self.current_camera_angle

      future = self.pick_up_moving_gear_client.call_async(
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

  # ===========================================================
  #                  conveyor belt functions
  # ===========================================================
  
  def enable_conveyor_service(self, enable: bool):
      """
      Calls the enable_conveyor callback
      """
      self.get_logger().info(
          ("Enabling " if enable else "Disabling ") + "the conveyor belt"
      )

      request = EnableConveyor.Request()
      request.enable = enable

      future = self.enable_conveyor_client.call_async(
          request
      )

      rclpy.spin_until_future_complete(self, future, timeout_sec=8)

      if not future.done():
          raise Error("Timeout reached when calling enable_conveyor service")

      result: EnableConveyor.Response
      result = future.result()

      if not result.success:
          raise Error(f"Unable to enable the conveyor belt")

  def set_conveyor_state_service(self, speed: float, direction: float):

      """
      Calls the set_conveyor_state callback
      """
      self.get_logger().info(
          "Moving the conveyor"+
          ("forward" if direction == 0 else "backward")+
          f"at a speed of {speed}",
      )
      request = SetConveyorState.Request()
      request.speed = float(speed)
      request.direction = int(direction)

      future = self.set_conveyor_state_client.call_async(
          request
      )

      rclpy.spin_until_future_complete(self, future, timeout_sec=20)

      if not future.done():
          raise Error("Tiemout reached when calling set_conveyor_state service")

      result: SetConveyorState.Response
      result = future.result()

      if not result.success:
          raise Error(f"Unable to move the conveyor belt")

  # ===========================================================
  #                  get information functions
  # ===========================================================
      
  def call_get_joint_positions(self):
      """
      Calls the get_camera_angle callback
      """
      self.get_logger().info("Getting joint positions")

      future = self.get_joint_positions_client.call_async(GetJointPositions.Request())

      rclpy.spin_until_future_complete(self,future,timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when getting joint positions")

      result: GetJointPositions.Response
      result = future.result()

      self.current_joint_positions = result.joint_positions

  def call_get_camera_angle(self):
      """
      Calls the get_camera_angle callback
      """
      self.get_logger().info("Getting camera angle")

      future = self.get_camera_angle_client.call_async(GetCameraAngle.Request())

      rclpy.spin_until_future_complete(self,future,timeout_sec=2)

      if not future.done():
          raise Error("Timeout reached when getting camera angle")

      result: GetCameraAngle.Response
      result = future.result()

      self.current_camera_angle = result.angle
    
  # ===========================================================
  #               move to position/pose functions
  # ===========================================================

  def call_move_to_named_pose_service(self, named_pose: str):
      """
      Calls the move_to_named_pose callback
      """
      self.get_logger().info(f"Moving to {named_pose}")

      request = MoveToNamedPose.Request()

      request.pose = named_pose

      future = self.move_to_named_pose_client.call_async(
          request
      )

      rclpy.spin_until_future_complete(self, future, timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when calling move_to_named_pose service")

      result: MoveToNamedPose.Response
      result = future.result()

      if not result.success:
          self.get_logger().error(f"Unable to move to pose: {named_pose}")
          raise Error("Unable to move to pose")

  def call_move_to_position_service(self, p: Point, rot: float = 0.0):
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

  # ===========================================================
  #                 joint movment functions
  # ===========================================================

  def call_move_to_joint_position(self, target_position : list):
    """
    Calls the move_to_joint_position callback
    """
    self.get_logger().info("Moving to joint position: "+", ".join([str(val) for val in target_position]))

    request = MoveToJointPosition.Request()
    
    for i in range(7):
        request.joint_positions[i] = target_position[i]

    future = self.move_to_joint_position_client.call_async(request)

    rclpy.spin_until_future_complete(self,future,timeout_sec=10)

    if not future.done():
        raise Error("Timeout reached when moving to joint position")

    result: MoveToJointPosition.Response
    result = future.result()

    if not result.success:
        raise Error("Unable to move to the given joint positions")
    
  def call_rotate_single_joint(self,joint : int, angle : float, radians : bool):
      """
      Calls the rotate_single_joint callback
      """
      self.get_logger().info(f"Rotating joint {joint} by {angle} "+("pi" if radians else "degrees")+" clockwise")

      request = RotateSingleJoint.Request()
      if radians:
        if angle>pi:
            angle=angle - 2*pi
        elif angle < -1 * pi:
            angle = angle + 2*pi
      else:
        if angle>180:
            angle=angle-360
        elif angle < -180:
            angle = angle +360

      request.joint = joint
      request.angle = float(angle)
      request.radians = radians
      
      future = self.rotate_single_joint_client.call_async(request)

      rclpy.spin_until_future_complete(self,future,timeout_sec=10)

      if not future.done():
          raise Error("Timeout reached when calling rotate_single_joint service")

      result: RotateSingleJoint.Response
      result = future.result()

      if not result.success:
          raise Error("Unable to rotate joint to given angle")
