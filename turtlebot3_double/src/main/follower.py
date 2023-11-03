#!/usr/bin/env python3

import rospy
import sys
import os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))


from classes.Sensor import Sensor

# ROS node initialization
rospy.init_node('turtlebot_controller', anonymous=True)
sensor = Sensor()

# Keep the node running
rospy.spin()