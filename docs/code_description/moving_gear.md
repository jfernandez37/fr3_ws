# moving_gear.py

## Overview

This file has the code for detecting a gear at different times and calculating a linear function as to where the gear will be at a time t.

## Important functions

* run

    The main loop in this function loops until the gear is found twice. The gear is found using the FindObject class. Then, using the OjbectDepth class, the position coordinates for the gear is found and all coordinate and time values are saved. If the function runs more than 5 times, the function returns with empty values and the gear was not found

* point_from_time

    This function takes in a float value for the time. Then, using the values found using the run function, it finds the x and y values of where the gear will be at that time.

* distance_to_point

    This function takes in a point and returns the distance between the camera and the center of the gear. This is used to find the values for the distance formula

* distance_formula

    This function uses the distance_to_point and point_from_time functions to find the slope and the intercept of the line of the gear moving.