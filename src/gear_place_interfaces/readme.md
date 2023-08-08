# gear_place_interfaces
ROS2 package containing all of the services and messages necessary for the ROS2 services.

## Services
* `move_cartesian` - XYZ values for the movement, max_velocity for the maximum velocity of the movement, and acceleration to control how fast the robot accelerates.
* `move_to_named_pose` - Takes in the name of the pose and if it is found in robot_commander.hpp, the robot will move to that pose.
* `pick_up_gear` - XYZ values for the cartesian movements and the width of the object for the grasp function. These will be used to move to the gear and pick it up.
* `move_to_conveyor` - XYZ value to move the robot to the conveyor belt. The robot then moves in the x and y direction, goes down to the conveyor belt, releases the gear, and moves back up.
* `move_to_position` - Moves to an XYZ location with a given rotation for the gripper.