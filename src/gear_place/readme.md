# gear_place
ROS2 package containing all of the code for controlling the robot and using the Realsense camera to identify the gear

## Directories
### gear_place
* `find_object.py` - Contains all of the code to use the depth image from the Realsense camera to identify the gear using OpenCV contours and returns the pixel coordinates of the center of the gear
* `gear_place.py` - Contains all of the code to call services like move_cartesian or move_to_named_pose
* `object_depth.py` - Contains all of the code to use the pointoud to find the distance between the camera and the top of the gear
* `transform_utils.py` - Contains all functions to convert transforms to different forms
