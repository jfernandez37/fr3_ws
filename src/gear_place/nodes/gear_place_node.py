#!/usr/bin/env python3

import rclpy
from gear_place.gear_place import GearPlace, Error
def main(args=None):
    rclpy.init(args=args)
    
    try:
        supervisor = GearPlace()
        
    except Error as e:
        print(e)

if __name__ == '__main__':
    main()