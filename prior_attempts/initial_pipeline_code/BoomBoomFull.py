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
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils import rgb_from_ros
import numpy as np
import imutils
import os
import roslaunch

LEFT_TRIM = 0
RIGHT_TRIM = 0
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("num_objects_detected",Int32, queue_size=10)
    print("__Init")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/riabot/camera_node/image/compressed",CompressedImage,self.callback)
    self.num_obj_detected = 0
  def callback(self,data):
    print("Call back")

    center_circle = (0,0)

    myimage = rgb_from_ros(data)

    # Shantha: Full image size i captured is 1280 x 720.
    # We want a 1200 x 300 cropped view in the bottom center of the full screenshot
    myimage = myimage[40:1240, 420:720]
    cv2.imwrite("cropped_ducks.jpg", myimage)

    frame = myimage
    im = frame
    frame = cv2.cvtColor(myimage, cv2.COLOR_RGB2BGR)

    # Convert BGR to HSV
    hsv_blue = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of yellow color in HSV
    lower_blue = np.array([51, 97, 100])
    upper_blue = np.array([56, 86, 100])

    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask_blue)
    res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
    cv2.imwrite('frame.jpg',frame)
    cv2.imwrite('mask.jpg',mask_blue)
    hsv_blue[mask_blue > 0] = ([56, 86, 100])
    cv2.imwrite('hsv_yellow.jpg',hsv_blue)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=60)
    circles_frame = frame.copy()

    # ensure at least some circles were found
    if circles is not None:
        self.num_obj_detected = 1
        print("Done! Found atleast one ducky")
    if circles is None:
        self.num_obj_detected = 0
        print("Done! No ducky found")

  def start(self):
	print("num obj detected", self.num_obj_detected)

    robot = Robot.Robot(left_trim=0, right_trim=0)

    while not rospy.is_shutdown():
		rate = rospy.Rate(2)
		rate.sleep()
		if self.num_obj_detected == 0:
        		robot.forward(100, 0.5)
        if self.num_obj_detected >= 1:
			    robot.stop()

if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    ic.start()
