#!/usr/bin/env python
from __future__ import print_function

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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("num_objects_detected",Int32, queue_size=10)
    print("__Init")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/riabot/camera_node/image/compressed",CompressedImage,self.callback)

  def callback(self,data):
    print("Call back")

    def get_the_image(data):
          myimage = rgb_from_ros(data)
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
          def distance_to_camera(knownWidth, focalLength, perWidth):
            return (knownWidth * focalLength) / perWidth
          KNOWN_DISTANCE = 12.0
          KNOWN_WIDTH = 1.5
#          image = cv2.imread("duckie_calibration.jpg")
#          marker = find_marker(image)
#          focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
          ###############
          cnts = find_marker_altered(hsv_blue)
          cnts = list(sorted(cnts, key = cv2.contourArea, reverse=True))[:30] #22
          detected_obj_list=[]
          for c in cnts:
            bbox = cv2.boundingRect(c)
            x,y,w,h = bbox
            if w<20 or h < 35 or w*h < 20 or w > 145:
              pass
            else:
              peri = cv2.arcLength(c, True)
              approx = cv2.approxPolyDP(c, 0.02 * peri, True)
              print("Hi", len(approx))
              if len(approx) == 4:
                pass
              if len(approx) > 4 and len(approx) <= 12: 
                marker = approx
    detected_obj_list.append(marker) 
                #inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
                cv2.drawContours(frame, [marker], -1, (0, 255, 0), 2)
          print(len(detected_obj_list))
    image_to_write = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
          try: 
            os.remove("edge_detected.jpg")
          except:
            pass
          cv2.imwrite( "./edge_detected.jpg", image_to_write)
          print("Done!")
    self.image_pub.publish(len(detected_obj_list))
        #print("Ctrl C now")
        #       cv2.imshow("Image window", myimage)
        #       cv2.waitKey(3)

          #try:
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(myimage, "bgr8"))
          #except CvBridgeError as e:
               #  print(e)

    get_the_image(data)

def main(args):
  print("main")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
    while not rospy.core.is_shutdown():
      rospy.rostime.wallsleep(0.5)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
