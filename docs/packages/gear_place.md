# gear_place
ROS2 package containing all of the code for controlling the robot and using the Realsense camera to identify the gear

## Directories
### gear_place
* `find_object.py` - Contains all of the code to use the depth image from the Realsense camera to identify the gear using OpenCV contours and returns the pixel coordinates of the center of the gear. For different depths, thresh value needs to be changed to recognize the gear
* `moving_gear.py` - Contains all of the code used to detect the moving gear and give the coordinates for the fr3 to move to the correct position
* `gear_place_classes.py` - Contains all of the code to call services like move_cartesian or move_to_named_pose
* `object_depth.py` - Contains all of the code to use the pointoud to find the distance between the camera and the top of the gear
* `transform_utils.py` - Contains all functions to convert transforms to different forms
* `multiple_gears.py` - Contains class for detecting multiple gears using the Realsense2 camera. Also can detect gears at different heights

### include/gear_place
* `motion_generator.hpp` - Contains all of the code for the control loop like move cartesian
* `robot_commander.hpp` - Contains all class variables, function prototypes, and includes for the robot_commander
* `robot_transformations.hpp` - Contains all of the functions for poses like multiplying the pose and building the pose

### launch
* `gear.launch.py` - Launch file for the workspace. Starts all of the nodes and publishers

### nodes
* `find_object_node.py` - Node used for testing the realsense gear detection using the depth camera
* `gear_place_node.py` - Node that combines everything and moves the robot to above the gear, picks it up, moves to the conveyor belt, and places the gear on the conveyor belt
* `moving_gear_node.py` - Node for testing the detection of a moving gear. Also estimates the distance from the center of the gear to the center of the camera
* `point_cloud_node.py` - Node for testing the point cloud. The points are read in and then printed for a hard coded coordinate
* `multiple_gears_node.py` - Node for testing the multiple_gears class. Similar to find_object_node
* `robot_commander_node.cpp` - Starts the robot commander so the robot can be moved

### src
* `robot_commander.cpp` - Contains all of the code for moving the robot like move_robot_cartesian, pick_up_gear, and the gripper control functions
