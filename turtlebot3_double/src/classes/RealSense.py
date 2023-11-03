#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from apriltag_ros.msg import AprilTagDetectionArray  # Import AprilTag messages

class RealSense:

    # Constructor:
    def __init__(self):
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscriber:
        self.imageSub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depthSub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        # Data Members of this class:
        self.depth_image = None
        self.rgb_image = None

        self.depth = None

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

    # Callback function for the subscriber:
    def depth_callback(self, msg):

        # Store the image in the data member:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        

        # Store the image in the data member:
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to HSV format:         
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the blue color
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])

        lower_yellow = np.array([20, 100, 100])  # Adjust these values as needed
        upper_yellow = np.array([30, 255, 255])  # Adjust these values as needed

        lower_red  = np.array([160, 100, 20])
        upper_red = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i, c in enumerate(contours):
            # Calculate the area of each contour:
            area = cv2.contourArea(c)

            # Ignore contours that are too small or too large:
            if area < 6789 or 100000 < area:
                continue

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Get the center, width, height, and angle of the bounding rectangle:
            cx, cy = int(rect[0][0]), int(rect[0][1])
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)
            depth = 0.285
            center = (cx, cy)

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rotated rectangle
            cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)

            angle = rect[-1]

            if angle > 45:
                angle -= 90
            
            print('with orientation angle:', angle)


        # for contour in contours:             

        #     if cv2.contourArea(contour) > 6789:

        #         M = cv2.moments(contour)                

        #         cx = int(M['m10'] / M['m00'])                 

        #         cy = int(M['m01'] / M['m00'])

        #         cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)

        #         # Compute the minimum area bounding rectangle
        #         rect = cv2.minAreaRect(contour)
        #         box = cv2.boxPoints(rect)
        #         box = np.int0(box)

        #         # Draw the rotated rectangle
        #         cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)

        #         # Extract the angle of the rotated rectangle
        #         angle = rect[-1]

        #         if angle > 45:
        #             angle -= 90

        #         print('with orientation angle:', angle)



        #         # x, y, w, h = cv2.boundingRect(contour)

        #         # cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        #         # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)  # (0, 255, 0) is the color, 2 is the thickness

        #         # self.depth = self.depth_image[cy][cx]

        #         # cv2.putText(cv_image, str(self.depth), (cx , cy ), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #         # print(self.depth)

        cv2.imshow('Detected Object', cv_image)

        cv2.waitKey(1)

    def aruco_callback(self, msg):
        rospy.sleep(1.0)
        # Store the image in the data member:
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the ROS message to an OpenCV image:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers:
        markerCorners, markerIds, _= cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if markerIds is not None:
            ########################
            #This function is used to get the rotation matrix and translation matrix 
            #for reference: https://docs.opencv.org/4.8.0/d9/d6a/group__aruco.html#ga3bc50d61fe4db7bce8d26d56b5a6428a
            marker_size = 0.2          #replace with real marker size

            fx = 0.9187401733398438     #focal length in x axis
            fy = 0.9183084716796875      #focal length in y axis
            cx = 0.6472181396484375            #principal point x
            cy = 0.3458296203613281            #principal point y

            camera_matrix = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0, 0, 1]], dtype=np.float64)

            
            k1 = 0
            k2 = 0
            p1 = 0
            p2 = 0
            k3 = 0
            
            dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, marker_size, camera_matrix, dist_coeffs)
            
            self.translation = tvecs
            self.rotation = rvecs
            

            #now use rvecs and tvecs for controller
            ########################

            cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

            # Example: Taking the depth value of the first detected marker's center
            self.center_x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) / 2)
            self.center_y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) / 2)

            # Get the depth value from the depth image:
            # self.depth = self.depth_image[self.center_y][self.center_x]
            self.depth = math.sqrt(self.translation[0][0][0]*self.translation[0][0][0] + self.translation[0][0][1]*self.translation[0][0][1])

            # Display the value on the image:
            cv2.putText(cv_image, str(self.depth), (self.center_x, self.center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw a circle at the center of the marker:
            cv2.circle(cv_image, (self.center_x, self.center_y), 5, (0, 0, 255), -1)

        else:
            print("can not DETECT")



        # Now you can visualize the image with detected markers using OpenCV
        cv2.imshow('Detected ArUco markers', cv_image)
        cv2.waitKey(1)  # Display the image for a short duration (1 ms). This keeps the display updated.