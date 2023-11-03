#!/usr/bin/env python3

import rospy
import cv2
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))


from classes.RealRobot import RealRobot

# ROS node initialization
rospy.init_node('real_sense', anonymous=True)
realRobot = RealRobot()

# Keep the node running
rospy.spin()