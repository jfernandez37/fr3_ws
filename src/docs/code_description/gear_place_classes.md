# gear_place_classes.py

## Overview

This file has the classes which is used for the node to call the services. Both of the main python classes are in this file: GearPlace and ConveyorClass.

## Important functions

* _call_move_to_named_pose_service

    This function calls the move_to_named_pose service. It takes in the name of a pose and will move to the pose under that name if it exists

* _call_move_cartesian_service

    This function calls the move_cartesian service. It takes in xyz coorinates, a maximum velocity value, and an acceleration value. The robot will then move until the xyz position is achieved.

* _call_pick_up_gear_service

    This function starts by creating an instance of the FindObject class. It runs until a gear is found. If it does not find a gear within the first 5 scans, it will move to a new position and start scanning again. 
    
    When a gear is found, an instance of ObjectDepth is created with the pixel coordinates of the center of the gear passed in. The object_depth class will find the xyz coordinates from the center of the camera to the center of the gear using the pointcloud. Once these coordinates are found, the pick_up_gear service is called and the gear is picked up.

* _call_put_gear_down_service

    This function calls the put_gear_down service. It passes in the higher z value between where the gear was picked up and the z value of the table.

* _call_pick_up_moving_gear_service

    This function first makes an instance of the MovingGear class. This runs until a gear is found. The MovingGear class gives the slope and the intercept of the line which the gear is traveling on. This way, it can be calculated where the arm and the gear can meet. 
    
    Once this time is calculated, a request is made for the pick_up_moving_gear service. There are two possibilites for the coordinates. If the gear is found, but does not seem to be moving, the gear will just be picked up like usual. If the gear is found to be moving, the calculated position is passed in and the gear is picked up.

* _call_pick_up_multiple_gears

    This function starts by establishing where the robot needs to move to scan the surface below. Then, it loops through these positions. In the loop, three scans are performed. Each scan finds all of the gears that it can, passes the pixel coordinates of the centers of the gear into ObjectDepth, and saves all of the valid coordinates in relation to the starting position and the gears' associated radii.

    The robot then performs the move established at the beginning. After all positions are moved to and scanned, the identical gears are removed. Also, any gears which are too far from the starting point are discarded, as they would be invalid. If no gears are found, the scan restarts. If gears are found, the robot then moves to the original position.

    The robot then loops through each gear that was found. The move is calculated from the current position to the next position, so the robot does not need to return to the home position between each gear. Then, the color of the gear is output. This is found by using the radius of the gear. The gripper then opens, the robot moves above the gear, a final scan is done to confirm the coordinates and get a more accurate grasp on the gear. The gear is then picked up and put down.

* _call_open_gripper_service

    This function calls the open_gripper service. This opens the gripper to just below the maximum width.

* _call_pick_up_gear_coord_service

    This function skips the camera code to pick up the gear. The coordinates are passed in and the gear is picked up at this position. Also, you can turn the offset on or off depending on if you are calculating the move from the previous gear center, in which the offset would already be accounted for.

* _call_put_gear_down_camera

    This function uses the pointcloud from the Realsense2 camera to detect how far the surface below the gear is from the gear. Then, it uses this z value to put the gear down. If the calculated value is lower than the height that it was picked up at, the higher of the heights will be used to avoid collision.

* _call_put_down_force

    This function uses the tourque control of the FR3 robot to detect when the gear has hit the surface. It moves down a centimeter at a time and tests if contact has been made with the surface. If it has, the robot rises and if not, the robot continues to move down.

* _call_move_above_gear

    The code for this function is similar to pick up moving gear. However, instead of trying to pick up the gear, the end effector tracks the gear from above. This is to test the coordinates for a moving gear.

## Important notes

 - When the robot moves to the "above_conveyor" position, the camera is rotated 90 degrees. Since the camera outputs xyz coordinates, the coordinates coming out of the camera are not able to be directly passed into any service using move_cartesian. For all functions used when the camera is rotated (_call_move_above_gear and _call_pick_up_moving_gear), the coordinates rotation is accounted for using the rotated boolean variable. When set to true, the rotation is taken into account.