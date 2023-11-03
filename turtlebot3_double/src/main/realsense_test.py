#!/usr/bin/env python

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


from classes.RealSense import RealSense

# ROS node initialization
rospy.init_node('real_sense', anonymous=True)
realSense = RealSense()

# Keep the node running
rospy.spin()
