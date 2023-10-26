#!/usr/bin/env python3

import rclpy

from gear_place.gear_place_classes import GearPlace, Error

from time import sleep

from math import pi


def main(args=None):
	rclpy.init(args=args)
	gear_width = 0.0095
	try:
		supervisor = GearPlace()
		supervisor.wait(5)
		supervisor.call_move_cartesian_smooth_service(0.03,0.04,0.01,0.2,0.15)
	except Error as e:

		print(e)

if __name__ == "__main__":
	main()