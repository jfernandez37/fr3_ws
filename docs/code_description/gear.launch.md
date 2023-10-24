# gear.launch.py

## Overview

This file launches everything needed for the robot to run. 

## Important functions

* launch_setup

    First, this function gets the urdf file from gear_place_description and gets the robot description from it. Then, the robot_state_publisher is declared. Then, the realsense node is declared. This launches the camera with the correct settings. It is needed to get the PointCloud and the depth camera correctly set up.

    Next is the RVIZ node, which is needed to run rviz if it is set to true. Then, the robot_commander_node is set. This includes all of the code that moves the robot. The conveyor_node is needed to control the conveyor belt. The last node is the supervisor node, which runs the main node used to call services.

* generate_launch_description

    This function launches all of the nodes including the RVIZ node if it is set to true using `rviz:=true`.