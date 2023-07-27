#!/usr/bin/env python3

import rclpy
from gear_place.gear_place import GearPlace, Error
def main(args=None):
    rclpy.init(args=args)
    
    try:
        supervisor = GearPlace()
        supervisor._move_to_named_position_service("home")
        supervisor._move_cartesian_service(0.05,0.03,-0.02,0.15,0.2)
        supervisor._move_cartesian_service(-0.05,-0.03,0.02,0.15,0.2)
    except Error as e:
        print(e)

if __name__ == '__main__':
    main()