# multiple_gears.py

## Overview

This file has the code for detecting multiple gears.

## Important functions

* camera_cb

    This is a callback which is called when the camera info is recieved. This is so it is known when the camera is connected.

* closest_to_circle

    This function loops through the contours and returns an array with the indicies of the of the contours which are gears. It does this this by detecting which of the contours is closest to a circle.

* remove_bad_contours

    This function removes any contours which are not larger than a minimum size. Also, all of the contours which are not convex are removed.

* listener_callback

    This is the function with all of the main contour detection code. It is a callback which runs when a depth image is recieved. First, it converts the ROS2 image message to a cv image. Then, the contrast of the image is scaled to make the gear easier to detect. For pixels which are white, they are converted to black, which allows the gear to be detected at different heights.

    A gaussian blur is then applied multiple times. Then, the thresholds are looped through so gears at different heights are detected. The contours are found for each threshold and they are filtered through. The center of the gears are found and saved. Also, points on the edge of the gear are saved so the radius measurements can be found to detect which gear is being picked up.

### Important notes

In this file, there are two classes, MultipleGears and MultipleGearsHigh. MultipleGearsHigh will use the color image to detect the gears from a higher position, but it is not finished yet, so it is very similar to MultipleGears.