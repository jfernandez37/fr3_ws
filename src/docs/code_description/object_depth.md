# object_depth.py

## Overview

This file has the code for detecting the distance between a point in the image and the center of the camera.

## Important functions

* dist_between_points

    This function takes two points and returns the euclidean distance between the two points. This is only done for the x and y coordinates, as height distance is not important here.

* read_points

    This function is from Intel to read the points from the PointCloud.

* _get_struct_fmt

    This function is also from Intel and is needed for the read_points function

* listener_callback

    This function takes in the ROS2 PointCloud. Then, it loops through the points passed in and passes them into the read_points function. For each point, it also finds the radius coordinates. This means that the measurements for the radius can be found for the gear.