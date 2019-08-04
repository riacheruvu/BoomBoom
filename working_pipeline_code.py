#!/usr/bin/env python
from __future__ import print_function
import cv2
import numpy as np
import imutils
import os
from PIL import Image
from scipy.spatial import distance as dist
import time
import Robot
import roslib
import sys
import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils import rgb_from_ros
import numpy as np
import imutils
import os
import roslaunch

#NEED TO CONFIRM: Use default values for trim
LEFT_TRIM = 0
RIGHT_TRIM = 0

class image_converter:

  def __init__(self):
    #Here, we subscribe to the CompressedImage Stream; we do not need a Publisher, as all our code is one file
    #The queue_size is a parameter that was carefully tuned, a large incoming message queue size to 20 causes the program to not function properly (e.g. detect same duckie 5 times)
    #Reducing the queue_size makes the robot take a long time for the object detection, so queue_size = 2 is a relatively perfect balance.
    self.image_sub = rospy.Subscriber("/riabot/camera_node/image/compressed",CompressedImage,self.callback, queue_size=2)
    #Instantiate a robot with default trim values
    self.robot = Robot.Robot(left_trim=0, right_trim=0)
    #We use this parameter in order to make sure that even if the same duckie 
    #is detected twice, we don't perform the stop and swerve logic twice - this flag helps in most scenarios
    self.obj_detected = 0

  def callback(self,data):
    center_circle = (0,0)
    myimage = rgb_from_ros(data)
    #Crop image to limit window of duckiebot
    myimage = myimage[100:500, 80:800]
    #Output image for debug purposes
    cv2.imwrite("cropped_ducks.jpg", myimage)
    frame = myimage
    #Convert RGB image to BGR
    frame = cv2.cvtColor(myimage, cv2.COLOR_RGB2BGR)
    # Convert BGR to HSV
    hsv_blue = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of yellow color in HSV
    lower_blue = np.array([51, 97, 100])
    upper_blue = np.array([56, 86, 100])

    # Threshold the HSV image to only get yellow colors
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)

    # Optionally, we can bitwise-AND the mask and the original image
    #We don't use this code here, but we experimented with it, and keep it for the reader's review
    #res = cv2.bitwise_and(frame, frame, mask_blue)
    #Convert the image from HSV to BGR
    #res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
    # Here, we threshold the image further, since mask_blue is entirely black

    hsv_blue[mask_blue > 0] = ([56, 86, 100])
    #Convert the thresholded image to gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    #Implement the HoughCircles algorithm to find the duckie(s)'s heads, which are circles
    #NEED TO CONFIRM: We found better performance when using HoughCircles on the transformed image compared to the original
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=60)
    circles_frame = frame.copy()

    #Here, we ensure at least some circles were found, and output an image containign the circles around the original objects
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            center_circle = center
            # circle center
            cv2.circle(circles_frame, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(circles_frame, center, radius, (255, 0, 255), 3)
            cv2.imwrite("circle.jpg", circles_frame)
    
    # Here, we ensure at least some circles were found and implement the stop and swerve logic
    if circles is not None:
        print("Found duckies")
    if self.obj_detected == 0:
        #If we detect an object, we stop, move right, move a little forward, move left, and move forward again, to simulate swerving
        #We move forward just a bit, even if we detect a duckie, to simulate real-life conditions, where cars can't stop right away
        #before they try to swerve.
        self.robot.forward(75, 1.0) 
        self.robot.stop()
        self.robot.right(90, 0.5)
        self.robot.forward(75, 1.0)
        self.robot.left(100, 0.3)
        self.robot.forward(75, 0.5)
        #Set the flag, so even if we detect the same duckie again, we try not to repeat the same logic
        self.obj_detected = 1
    else:   
    #If no duckies are detected, we just move forward
        print("No duckies detected")
        self.robot.forward(75, 5.5)
if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.spin()    
