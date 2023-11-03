# Import libraries:
import rospy

# Importing messages:
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Controller:

    # Constructor: def __init__(self, stdscr):
    def __init__(self):

        # Publisher:
        self.cmd_vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)

        # Subcriber for the controller:
        rospy.Subscriber('joy', Joy, self.joy_callback)

        # Initializing the Twist message:
        self.move_cmd = Twist()

    # Function to move the robot forward:
    def move_forward(self, linear_speed = 0.2, angular_speed = 0.2):
        self.move_cmd.linear.x = linear_speed
        self.move_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to move the robot backward:
    def move_backward(self, linear_speed = -0.2, angular_speed = -0.2):
        self.move_cmd.linear.x = linear_speed
        self.move_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to turn the robot left:
    def turn_left_right(self, angular_speed = 0.2):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.move_cmd)

    # Function to stop the robot:
    def stop(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)

    # Callback function for the subscriber:
    def joy_callback(self, msg):
        if msg.buttons[5] == 1:
            self.stop()
        else:
            if msg.axes[5] == 0.5:
                self.move_cmd.linear.x = msg.axes[1]*0.05
                self.move_cmd.angular.z = msg.axes[0]*0.05
                self.cmd_vel_pub.publish(self.move_cmd)
            else:
                self.move_cmd.linear.x = msg.axes[1]
                self.move_cmd.angular.z = msg.axes[0]
                self.cmd_vel_pub.publish(self.move_cmd)