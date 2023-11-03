# Import
import rospy
import cv2 
print(cv2.getBuildInformation())
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import math
import time

# Import the Sensor message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sklearn.linear_model import LinearRegression
from sensor_msgs.msg import LaserScan

#For documentation only:
#Max linear speed = 0.26
#Max rational speed = 1.82 rad/s

class Sensor:

    # Constructor:
    def __init__(self):
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscriber:
        self.image_sub = rospy.Subscriber('/tb3_0/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/tb3_0/camera/depth/image_raw', Image, self.depth_callback)
        self.laser_sub = rospy.Subscriber('/tb3_0/scan', LaserScan, self.laser_callback)

        # Publisher:
        self.cmd_vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)

        # Data Members of this class:
        self.depth_image = None
        self.rgb_image = None

        # ArUco Setup:
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Initializing the Twist message:
        self.move_cmd = Twist()

        # Initializing the translation and rotation vec
        self.translation = None
        self.rotation = None

        # Initializing depth, center x and y axis value
        self.depth = None
        self.center_x = None
        self.center_y = None

        self.last_translations = []  # List to store translation vectors
        self.last_rotations = []     # List to store rotation vectors
        self.max_readings = 50       # Maximum number of readings to keep

        # Initillzing laser scan data
        self.laser_data = None

    # Laser call back function
    def laser_callback(self, msg):
        self.laser_data = msg

    # Callback function for the subscriber:
    def depth_callback(self, msg):

        # Store the image in the data member:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Callback function for the subscriber:
    def image_callback(self, msg):

        # Store the image in the data member:
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the ROS message to an OpenCV image:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to gray image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers:
        markerCorners, markerIds, _= cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        # If camera can detect ArUco marker
        if markerIds is not None:
            ########################
            #This function is used to get the rotation matrix and translation matrix 
            #for reference: https://docs.opencv.org/4.8.0/d9/d6a/group__aruco.html#ga3bc50d61fe4db7bce8d26d56b5a6428a
            marker_size = 0.23          #real marker size

            fx = 1.085595               #focal length in x axis
            fy = 1.085595               #focal length in y axis
            cx = 320                    #principal point x
            cy = 240                    #principal point y

            # Define camera matrix
            camera_matrix = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0, 0, 1]], dtype=np.float64)

            
            k1 = 0
            k2 = 0
            p1 = 0
            p2 = 0
            k3 = 0
            
            dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

            # Compute translation and rotation matrix of current position 
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, marker_size, camera_matrix, dist_coeffs)
            
            self.translation = tvecs
            self.rotation = rvecs

            # Update last known translation and rotation
            self.last_translations.append(self.translation)
            self.last_rotations.append(self.rotation)

            # Limit the size of the lists to the last 50 readings
            if len(self.last_translations) > self.max_readings:
                self.last_translations.pop(0)
            if len(self.last_rotations) > self.max_readings:
                self.last_rotations.pop(0)
            

            #now use rvecs and tvecs for controller
            ########################

            cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

            # Example: Taking the depth value of the first detected marker's center
            self.center_x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) / 2)
            self.center_y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) / 2)

            # Get the image width and height:
            height, width = cv_image.shape[:2]

            error_x = self.center_x - width / 2

            # Control stategy:
            # define Kp
            kp = 0.001
            angular_velocity = -kp*error_x

            # Get the depth value from the depth image:
            # self.depth = self.depth_image[self.center_y][self.center_x]
            self.depth = math.sqrt(self.translation[0][0][0]*self.translation[0][0][0] + self.translation[0][0][1]*self.translation[0][0][1])
                        
            if self.depth == 0.3 or self.center_x is None:
                linear_velocity = 0.0
            # In case it need to be going reverse
            elif self.depth < 0.3:
                # Define kp for linear velocity
                kp_linear = -0.25
                linear_velocity = -0.2 * self.depth * 2
            else:
                # Define kp for linear velocity
                kp_linear = 0.25
                linear_velocity = kp_linear*(self.depth)

            # get laser range data
            laser_range = self.laser_data.ranges
            
            # Check surrounding space to detect obstacle
            count = 0
            for i in range(len(laser_range)):
                if laser_range[i] < 0.18 and laser_range[i] > self.laser_data.range_min:
                    count = count + 1

            # If obstacle is detected, stop the robot
            if count >= 18:
                rospy.INFO("COLLISION")
                linear_velocity = 0.0
                angular_velocity = 0.0
            
            # Send command velocity to robot
            self.move_cmd.linear.x = linear_velocity
            self.move_cmd.angular.z = angular_velocity
            self.cmd_vel_pub.publish(self.move_cmd)

            # Display the value on the image:
            cv2.putText(cv_image, str(self.depth), (self.center_x, self.center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw a circle at the center of the marker:
            cv2.circle(cv_image, (self.center_x, self.center_y), 5, (0, 0, 255), -1)
        
        # When the follower robot lose sight of ArUco marker, predict the next moves of guider robot and go back on track
        else:
            
            # Stop the robot:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)

            # Using last detected position to calculate expected time to get there
            if self.translation is not None:
                durationLinear = self.depth/0.26
                print(durationLinear)
            else:
                durationLinear = 0

            # Using last detected position to calculate expected time to get there
            durationRotate = math.asin(abs(self.center_x)/self.depth)/1.82

            #Go to the position first
            startTime = time.time()
            while True:
                #Calculate the elapsed time
                elapseTime = time.time() - startTime

                linear_velocity = 0.26
                angular_velocity = 0

                self.move_cmd.linear.x = linear_velocity
                self.move_cmd.angular.z = angular_velocity
                self.cmd_vel_pub.publish(self.move_cmd)

                # Stop after reach expected pose
                if (elapseTime >= durationLinear+0.98):
                    print('break')
                    break
            
            # Stop the robot
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)

            # Get current time. Fix the angle
            startTime = time.time()
            while True:
                #Calculate the elapsed time
                elapseTime = time.time() - startTime
                
                linear_velocity = 0.0
                if self.center_x > 0:
                    angular_velocity = -1.82
                else:
                    angular_velocity = 1.82

                self.move_cmd.linear.x = linear_velocity
                self.move_cmd.angular.z = angular_velocity
                self.cmd_vel_pub.publish(self.move_cmd)


                # Stop after reach expected pose
                if (elapseTime >= durationRotate+0.5):
                    break
            
            # Stop the robot
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)

            #############################This is to calculate the trend to predict next location of the guider#####################################################

            # Calculate the trend in translation and rotation using linear regression
            trend_translation = self.calculate_trend(self.last_translations)
            trend_rotation = self.calculate_trend(self.last_rotations)

            # Predict the current translation and rotation
            predict_current_translation = self.predict_current_value(trend_translation, self.last_translations)
            predict_current_rotation = self.predict_current_value(trend_rotation, self.last_rotations)

            # Limit the size of the lists to the last 50 readings
            if len(self.last_translations) > self.max_readings:
                self.last_translations.pop(0)
            if len(self.last_rotations) > self.max_readings:
                self.last_rotations.pop(0)

            # Update last known translation and rotation
            self.last_translations.append(predict_current_translation)
            self.last_rotations.append(predict_current_rotation)

            current_distance = math.sqrt(predict_current_translation[0][0][0]*predict_current_translation[0][0][0] + predict_current_translation[0][0][1]*predict_current_translation[0][0][1])
            current_angular = predict_current_rotation[0][0][0] #Get yaw value

            #Get time for linear
            durationLinear = current_distance/0.26
            durationRotate = abs(current_angular)/1.82

            #Go to the position first
            startTime = time.time()
            while True:
                #Calculate the elapsed time
                elapseTime = time.time() - startTime

                linear_velocity = 0.26
                angular_velocity = 0

                self.move_cmd.linear.x = linear_velocity
                self.move_cmd.angular.z = angular_velocity
                self.cmd_vel_pub.publish(self.move_cmd)

                # Stop after reach expected pose
                if (elapseTime >= durationLinear+0.98):
                    print('break')
                    break
            
            # Stop the robot
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)

            # Get current time. Fix the angle
            startTime = time.time()
            while True:
                #Calculate the elapsed time
                elapseTime = time.time() - startTime
                
                linear_velocity = 0.0
                if current_angular > 0:
                    angular_velocity = -1.82
                else:
                    angular_velocity = 1.82

                self.move_cmd.linear.x = linear_velocity
                self.move_cmd.angular.z = angular_velocity
                self.cmd_vel_pub.publish(self.move_cmd)


                # Stop after reach expected pose
                if (elapseTime >= durationRotate+0.5):
                    break
            
            # Stop the robot
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)
            #########################################################################################

        # Now you can visualize the image with detected markers using OpenCV
        cv2.imshow('Detected ArUco markers', cv_image)
        cv2.waitKey(1)  # Display the image for a short duration (1 ms). This keeps the display updated.
    
    # Function is used to calculate the motion trend of the guider based on last 50 reading data
    def calculate_trend(self, data):
        # Calculate the trend (linear regression) for a given data set
        n = len(data)
        x = np.arange(n).reshape(-1, 1)
        y = np.array(data)

        model = LinearRegression()
        model.fit(x, y)

        trend = model.coef_[0]
        return trend

    # Function is used to predict the next position of the guider robot based on the calculated trend
    def predict_current_value(self, trend, last_data):
        # Predict the current value based on the trend
        n = len(last_data)
        current_value = last_data[-1] + (n * trend)
        return current_value




        