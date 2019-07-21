#Credit: Code reused and modified from https://raw.githubusercontent.com/pirobot/ros-by-example/master/rbx_vol_1/rbx1_vision/nodes/cv_bridge_demo.py

#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils import rgb_from_ros

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/riabot/camera_node/image/compressed",CompressedImage)
    print("__Init")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/riabot/camera_node/image/compressed",CompressedImage,self.callback)

  def callback(self,data):
    print("Call back")
    print(data)
    try:
       myimage = rgb_from_ros(data)
       print("RGB Image = ", myimage)
    #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print("Hark! I am an error", e)

#    (rows,cols,channels) = cv_image.shape
#    if cols > 60 and rows > 60 :
#      cv2.circle(cv_image, (50,50), 10, 255)
    print("About to write image")
    image_to_write = cv2.cvtColor(myimage, cv2.COLOR_RGB2BGR)
    cv2.imwrite( "./duckies.jpg", image_to_write)
    print("Ctrl C now")
    cv2.imshow("Image window", myimage)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("main")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
