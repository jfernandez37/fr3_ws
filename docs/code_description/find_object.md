# find_object.py

## Overview

This file has the code for detecting a single gear using the Intel Realsense2 camera.

## Important functions

* closest_to_circle

    This function loops through the contours and returns an array with the indicies of the of the contours which are gears. It does this this by detecting which of the contours is closest to a circle.

* remove_bad_contours

    This function removes any contours which are not larger than a minimum size and not smaller than a maximum. Also, all of the contours which are not convex are removed.

* listener_callback

    This is the function with all of the main contour detection code. It is a callback which runs when a depth image is recieved. First, it converts the ROS2 image message to a cv image. Then, the contrast of the image is scaled to make the gear easier to detect. For pixels which are white, they are converted to black, which allows the gear to be detected at different heights.

    A gaussian blur is then applied multiple times. Then, the thresholds are looped through so gears at different heights are detected. The contours are found for each threshold and they are filtered through. The center of the gears are found and saved. Also, points on the edge of the gear are saved so the radius measurements can be found to detect which gear is being picked up.

* ret_cent_gear

    Returns the x and y pixel coordinates for the gear.

* ret_edge_gear

    Returns the x and y pixel coordinates for a point on the edge of the gear.