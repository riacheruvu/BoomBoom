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
    frame = myimage
    #myimage = myimage[100:350, 100:1000]
    cv2.imwrite("cropped_ducks.jpg", myimage)
    #im = Image.open("Shantha_duck.jpg")
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
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            center_circle = center
            # circle center
            cv2.circle(circles_frame, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(circles_frame, center, radius, (255, 0, 255), 3)


    def find_marker(image):

      # convert the image to grayscale, blur it, and detect edges
      try:
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      except:
            pass
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      gray = cv2.GaussianBlur(image, (5, 5), 0)
      edged = cv2.Canny(gray, 35, 125)
      cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
      cnts = imutils.grab_contours(cnts)
      print("I found {} matching shapes".format(len(cnts)))
      c = max(cnts, key = cv2.contourArea)
      return cv2.minAreaRect(c)


    def find_marker_altered(image):
      # blur image, and detect edges
      gray = cv2.GaussianBlur(image, (5, 5), 0)
      edged = cv2.Canny(gray, 35, 125)
      cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
      cnts = imutils.grab_contours(cnts)
      print("I found {} matching shapes".format(len(cnts)))
      return cnts

    def find_distance(c):
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        immat = im.load()
        (X, Y) = im.size
        m = np.zeros((X, Y)) #Is this needed?
        m = np.sum(np.asarray(im), -1) < 255*3
        m = m / np.sum(np.sum(m))

        dx = np.sum(m, 0) 
        dy = np.sum(m, 1) 
        # expected values
        new_frame = frame.copy()
        middle_of_image_x = np.sum(dx * np.arange(X))
        middle_of_image_y = np.sum(dy * np.arange(Y))   
        print("x, y", cX, cY)
        print(middle_of_image_x, middle_of_image_y)
        print("X, Y")
        obj_center_dist = dist.euclidean((cX, cY), (middle_of_image_x, middle_of_image_y))
        print(obj_center_dist)  
        # draw the contour and center of the shape on the image
        cv2.drawContours(new_frame, [c], -1, (0, 255, 0), 2)
        cv2.circle(new_frame, (cX, cY), 7, (255, 255, 255), -1)

        cv2.putText(new_frame, str(cX - middle_of_image_x), (cX - 20, cY - 20), #str(obj_center_dist) + 
          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imwrite("new_distance_image" + str(c[1]) + ".jpg", new_frame)

        # show the image

    def distance_to_camera(knownWidth, focalLength, perWidth):
        return (knownWidth * focalLength) / perWidth

    KNOWN_DISTANCE = 12.0
    KNOWN_WIDTH = 1.5
    image = cv2.imread("duckie_calibration.jpg")

    marker = find_marker(image)
    focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
    ###############

    cnts = find_marker_altered(hsv_blue)
    cnts = list(sorted(cnts, key = cv2.contourArea, reverse=True))[:30] #22
    detected_obj_list=[]

    for c in cnts:
        bbox = cv2.boundingRect(c)
        x,y,w,h = bbox
        if w<20 or h < 45 or w*h < 20: # h < 50  or w > 1000 do h < 35 to get perfect detection on test_ducks, h < 50 for Shantha_duck
            pass
        else:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            print("Hi", len(approx))
            if len(approx) == 4:
                pass
            if len(approx) > 4 and len(approx) <= 12: 
                distance_param = cv2.pointPolygonTest(c,center_circle,False)
                print(distance_param)
                #if distance_param == 1.0:
                marker = approx 
               	#else: #See if you can refine this further!!
                detected_obj_list.append(marker) 
               	inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
               	cv2.drawContours(frame, [marker], -1, (0, 255, 0), 2) #0, 255, 0
               	print([marker])
                	#find_distance(c) 
    self.num_obj_detected = len(detected_obj_list)
    #self.num_obj_detected = num_obj_detected
    #robot = Robot.Robot(left_trim=0, right_trim=0)
      #if num_ob_detected >=1:
      #    robot.forward(150, 1.0)
      #rospy.spin()
    #rate = rospy.Rate(10)
    #print("num obj detected", self.num_obj_detected)
    #if self.num_obj_detected == 0:
	#robot.forward(100, 30.5)
    #if self.num_obj_detected >= 1:
    	#robot.stop() 
		
#image_to_write = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#try: 
#    os.remove("hi_newest_edge_detected.jpg")
#except:
#    pass
#cv2.imwrite( "hi_newest_edge_detected.jpg", image_to_write)
    print("Done!")



  def start(self):
	#print("num obj detected", self.num_obj_detected)
	robot = Robot.Robot(left_trim=0, right_trim=0)        
	while not rospy.is_shutdown():
		print("num obj detected", self.num_obj_detected)
		rate = rospy.Rate(2)
		rate.sleep()
		if self.num_obj_detected == 0:
        		robot.forward(100, 0.5)
			#robot.stop()
			#robot.forward(100, 2.5)
        	if self.num_obj_detected >= 1:
        	#robot.stop() 	
			robot.stop()
		#rospy.spin()
		#	sys.exit()
      #robot = Robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)
      #if num_ob_detected >=1:
      #    robot.forward(150, 1.0)
      #rospy.spin()
      #rate = rospy.Rate(10)
      #print(self.num_obj_detected)
      #if self.num_obj_detected == 0:
      #	robot.forward(100, 30.5)
      #if self.num_obj_detected >= 1:
      #	robot.stop() 
#      while not rospy.core.is_shutdown():
        #rospy.rostime.wallsleep(0.5)
 #       print(self.num_obj_detected)
  #      if self.num_obj_detected == 0:
   #     	robot.forward(100, 30.5)
    #    if self.num_obj_detected >= 1:
#		robot.stop()  
    #except KeyboardInterrupt:
     # print("Shutting down")
    #cv2.destroyAllWindows()

#def main(args):
#	ic = image_converter()
#	rospy.init_node('image_converter', anonymous=True)
#	while True:
#		try:
#			robot = Robot.Robot(left_trim=0, right_trim=0)
      #if num_ob_detected >=1:
      #    robot.forward(150, 1.0)
      #rospy.spin()
    #rate = rospy.Rate(10)
#			print("num obj detected", self.num_obj_detected)
#			if self.num_obj_detected == 0:
 #       			robot.forward(100, 30.5)
  #  			if self.num_obj_detected >= 1:
   #     			robot.stop() 
		
#			rospy.spin()
#		except KeyboardInterrupt:
#			print("Shutting down")
#			cv2.destroyAllWindows()
if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    ic.start()
    #main(sys.argv)
