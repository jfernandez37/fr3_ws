# Franka Emika FR3 ROS2 Implementation

## Overview

The purpose of this package is for the FR3 robot to identify a gear on a surface using the Intel Realsense2 camera, pick up the gear, moving the gear to the conveyor belt, and starting the conveyor belt. The gear can be stationary or moving at a constant speed.

## Installation

* Follow the [instructions to build libfranka from source](https://support.franka.de/docs/installation_linux.html#building-from-source)

* Install ROS2 Realsense `sudo apt install ros-$ROS_DISTRO-librealsense2*`

* Clone the workspace

    `git clone https://github.com/jfernandez37/fr3_ws.git`

* Go into the workspace

    `cd fr3_ws`

* Clone the conveyor belt package

  `git clone https://github.com/usnistgov/aprs-ros-conveyor.git src/aprs-ros-conveyor`

* Clone the humble port of franka_ros2

    `git clone https://github.com/mcbed/franka_ros2.git -b humble src/franka_ros2`

* Install dependencies

    `rosdep install --from-paths src -y --ignore-src -r --rosdistro $ROS_DISTRO`

## Running the program

First, build the workspace

`colcon build`

Then, source the workspace:

`. install/setup.bash`

Every node needs the camera to be running with the correct settings. So, run
 
`ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true pointcloud.stream_filter:=0 depth_module.profile:='640x480x30'`

Open another terminal, navigate to the workspace, and source it with `. install/setup.bash`:

There are four main nodes. Three of them are for testing and the final one is for running everything together with the robot.

* point_cloud_node

    This node tests the point cloud to find the depth of a given point. This point needs to be hardcoded. To run this node, use the command:

    `ros2 run gear_place point_cloud_node.py`

* find_object_node

    This node tests the gear detection code and combines it with the object depth code. This means it will detect the gear, find the center, and output the distance in XYZ from the center of the camera to the center of the gear. To run this node, use the command:

    `ros2 run gear_place find_object_node.py`

* moving_gear_node

    This node tests the moving gear detection. It uses multiple iterations of find_object and object_depth to calculate where the gear will be after a given amount of time. To run this node, use the command:

    `ros2 run gear_place moving_gear_node.py`

* launching everything

    To launch everything at once, including the robot, use the command:

    `ros2 launch gear_place gear.launch.py`

    To launch everything with rviz, use the command:

    `ros2 launch gear_place gear.launch.py rviz:=True`
  
    This automatically launches the camera with the correct settings, so you do not need to run the camera launch command above.

## Packages
* `gear_place` - Includes all code for moving the robot, identifying the gear with the realsense_camera, and the supervisors to control the robot
* `gear_place_description` - Includes all meshes, robot visuals, and urdf files for generating the robot description and visualizing the robot in rviz
* `gear_place_interfaces` - Includes all msg and srv for the ROS2 callbacks
