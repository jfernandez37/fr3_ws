# find_object.py

## Overview

This file contains the code for moving the robots. It contains all of the ROS2 callbacks used in the supervisor.

## Important functions

* RobotCommander

    The constructor starts by connecting to the robot and connecting to the gripper. Then it creates the ik solver and outputs general information about the robot. The three publishers are then created for the joint states, end effector pose, and the camera angle. The timers and services are then assigned to their callbacks.

* move_to_named_pose_cb_

    This function takes in a string. If this string is the name of a pose from named_joint_positions_ in robot_commander.hpp, then a motion generator will be built to move to that position. Otherwise, an error message will print saying that named pose does not exist and the call back will response with success as false.

* joint_state_publish_timer_cb_

    This takes the current positions, velocities, and efforts of each of the joints and adds them to the joint_state_msg_. Also, the names of the joints are added using the joint_names_ array from robot_commander.hpp. The values for the fingers are set to 0. The time stamp is then added and the message is published.

* ee_pose_publish_timer_cb_

    First, the frame_id is set to world and the time stamp is set to the current time. Then, using the solve_fk function, the end effector pose is found and the message is published.

* camera_angle_publish_timer_cb_

    The angle is gotten using the get_camera_angle function and then published.

* move_cartesian_cb_

    The request for this function takes xyz coordinates, a maximum velocity, and an acceleration. Then, it calls the move_robot_cartesian function using these values.

* move_robot_cartesian

    This function takes in xyz coordinates, a maximum velocity, and an acceleration. It creates a pointer for a cartesian_motion_generator, found in motion_generators.hpp. Then it tries to fill that generator with the variables that were passed into the function. After this, it uses a robot control loop to conduct this motion.

* move_cartesian_angle_cb_

    The request for this function takes xyz coordinates, a maximum velocity, an acceleration, and an angle. Then, it calls the move_robot_cartesian function using these values.

* move_robot_cartesian

    This function takes in xyz coordinates, a maximum velocity, an acceleration, and an angle. Then, the x and y coordinates are rotated to the passed in angle. It creates a pointer for a cartesian_motion_generator, found in motion_generators.hpp. Then it tries to fill that generator with the new variables and the variables that were passed into the function. After this, it uses a robot control loop to conduct this motion.

* put_down_force_cb_

    This function starts by taking the current angle of the camera. Also, the time value is set to one and the intial pose is set to the current joint positions so the robot can move back to the initial position after the motion is finished. One call of the put_down_force is then called with a sleep before and after. Then, a loop is entered where the robot constantly moves down 1 cm and 1.3 mm away from the base of the robot to accound for the put_down_force movement toward the robot.

    Each call of the put_down_force function call except for the first one are measured using std::chrono::high_resolution_clock so we can determine whether the gear has made contact with the surface. Once contact has been made, the gripper opens using the open_gripper function and the robot moves up one centimeter. Then, the robot returns to the joint positions the movement started at.

* put_down_force

    This function takes in the desired force as a parameter. It then loads the robot model, reads the current state of the robot, and builds an instance of a ForceMotionGenerator. Using a control loop, the motion generator is run.

* pick_up_gear_cb_

    This function starts by moving directly above the gear is it is not already. It then moves down to the height where it can grasp the gear. The gear is then grasped and it moves back to its original height. The gripper state is then checked and an error message is printed if the gear is not grasped.

* put_gear_down_cb_

    This function takes in a height value. It then moves the robot down to the given height with an offset up, opens the gripper, and moves the robot back up to the original position.

* pick_up_moving_gear_cb_

    Using the request values (xyz coordinates and object width), the robot moves down 3/4 of the way down to have a larger range. It then moves in the x and y directions. Finally, it moves down to the height of the gear, grasps it, and moves back up to the original height. The gripper state is then checked and an error message is printed if the gear is not grasped.

* open_gripper_cb_

    This function has an empty request. The open_gripper function is called. This callback is used so that the gripper can be opened from the python programs.

* open_gripper

    This function uses the Libfranka move function for the gripper and moves it to 1 cm less than the maximum width.

* grasp_object

    First, if the object is wider than the gripper's maximum width, an error is thrown. Then, the gripper attempts to grasp the object using the Libfranka grasp function. If the object is grapsed, the function ends. If not, the gripper opens, the robot moves down 1.5 cm, and it is attempted again. The maximum number of attempts is 3.

* get_camera_angle

    Uses the joint positions to find the camera angle.

* get_camera_angle_cb_

    The request for this service is empty. The response is set to the value of get_camera_angle().

* rotate_single_joint_cb_

    This function first gets the current joint positions. Then, replaces the chosen joint measurement with the updated one. If it is in radians, it is directly passed. If it is in degrees, the measurement is converted to radians and passed. Then, the angle is corrected to only move so that the camera cord is not stretched.

    A new motion generator is then made with the new joint positions and the motion generator is passed into a control loop. This will move the robot to the new position.