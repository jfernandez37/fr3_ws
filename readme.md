# FR3 ROS2 Implementation
Work space for Franka Emika FR3 robot ROS2 integration

## Packages
* `gear_place` - Includes all code for moving the robot, identifying the gear with the realsense_camera, and the supervisors to control the robot
* `gear_place_description` - Includes all meshes, robot visuals, and urdf files for generating the robot description and visualizing the robot in rviz
* `gear_place_interfaces` - Includes all msg and srv for the ROS2 callbacks

## Installation Instructions
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

## Important commands to run

* To start the camera with all options needed for getting the depth of the gear, run
  
    `ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true pointcloud.stream_filter:=0 depth_module.profile:='640x480x30'`

* To test the gear detection functions, run this command. `thresh_value` can be set to different values for different heights
  
    `ros2 run gear_place find_object_node.py --ros-args -p thresh_value:=160`
  
* To launch everything, run:
  
    `ros2 launch gear_place gear.launch.py rviz:=True`
  
    This automatically launches the camera with the correct settings, so you do not need to run the camera launch command above.
