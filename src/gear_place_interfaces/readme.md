# gear_place_interfaces
ROS2 package containing all of the services and messages necessary for the ROS2 services.

## Services
* `move_cartesian` - XYZ values for the movement, max_velocity for the maximum velocity of the movement, and acceleration to control how fast the robot accelerates.
* `move_to_named_pose` - Takes in the name of the pose and if it is found in robot_commander.hpp, the robot will move to that pose.
