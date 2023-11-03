#!/usr/bin/env python3

# Importing modules:
import rospy
import curses

# Importing messages:
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Guider:

    # Constructor:
    def __init__(self, stdscr):

        # Publisher:
        self.cmd_vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subcriber for the controller:
        # rospy.Subscriber('joy', Joy, self.joy_callbackoy_callback)
        
        # Initializing the Twist message:
        self.move_cmd = Twist()

        self.stdscr = stdscr
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.refresh()

        self.stdscr.addstr(0,10,"Hit 'q' to quit")
        self.stdscr.refresh()

    # Function to move the robot forward:
    def move_forward(self, linear_speed = 0.1):
        self.move_cmd.linear.x = linear_speed
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to move the robot backward:
    def move_backward(self, linear_speed = -0.1):
        self.move_cmd.linear.x = linear_speed
        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to stop the robot:
    def stop(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to control the turtlebot by keyboard:
    def keyboard_control(self):
        while True:
            key = self.stdscr.getch()
            if key == ord('w'):
                self.move_cmd.linear.x = 0.15
                self.move_cmd.angular.z = 0
            elif key == ord('s'):
                self.move_cmd.linear.x = -0.15
                self.move_cmd.angular.z = 0
            elif key == ord('a'):
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0.5
            elif key == ord('d'):
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = -0.5
            elif key == ord(' '): # space key to stop
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
            elif key == ord('q'): # to quit
                break

            self.cmd_vel_pub.publish(self.move_cmd)

        # Stop the robot before quitting
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    
