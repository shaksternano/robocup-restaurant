#!/usr/bin/env python

import math
import sys

import rospy

from tiago_controller import TiagoController

if __name__ == "__main__":
    degrees = float(sys.argv[1])
    rospy.init_node("test_rotate", anonymous=True)
    controller = TiagoController(wait_for_services=False)
    radians = math.radians(degrees)
    controller.rotate(radians)
