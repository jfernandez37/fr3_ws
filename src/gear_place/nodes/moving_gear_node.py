#!/usr/bin/env python3
import rclpy
from gear_place.moving_gear import MovingGear
from time import time
    
def main(args=None):
    rclpy.init(args=args)
    moving_gear = MovingGear()
    for i in range(12):
        print(f"After {round(moving_gear.times[1]+i/4,2)} seconds from starting time, the gear is ",str(moving_gear.point_from_time(moving_gear.times[1]+i/4)),"away from the center of the camera")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
