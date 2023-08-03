# FR3 ROS2 Implementation
Work space for Franka Emika FR3 robot ROS integration

## Packages
* `gear_place` - Includes all code for moving the robot, identifying the gear with the realsense_camera, and the supervisors to control the robot
* `gear_place_description` - Includes all meshes, robot visuals, and urdf files for generating the robot description and visualizing the robot in rviz
* `gear_place_interfaces` - Includes all msg and srv for the ROS2 callbacks

## Installation Instructions
* Follow the [instructions to build libfranka from source](https://support.franka.de/docs/installation_linux.html#building-from-source) 

* Clone the workspace

    `git clone https://github.com/jfernandez37/fr3_ws.git`

* Go into the workspace

    `cd fr3_ws`

* Clone the humble port of franka_ros2

    `git clone https://github.com/mcbed/franka_ros2.git -b humble src/franka_ros2`

* Install dependencies

    `rosdep install --from-paths src -y --ignore-src -r --rosdistro humble`
