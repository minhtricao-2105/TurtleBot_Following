#!/usr/bin/env python3

import rospy
import curses
import sys
import os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from classes.Guider_controller import Guider
from classes.Controller import Controller

rospy.init_node('turtlebot_controller', anonymous=True)

controller = Controller()

rospy.spin()
