#Citation to come!

# import the necessary packages
import numpy as np
import argparse
import cv2
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
 
# load the image
img = cv2.imread(args["image"])


#converting frame(img) from BGR (Blue-Green-Red) to HSV (hue-saturation-value)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#defining the range of Yellow color
yellow_lower = np.array([22,60,200],np.uint8)
yellow_upper = np.array([60,255,255],np.uint8)

#finding the range yellow colour in the image
yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

#Morphological transformation, Dilation        
kernal = np.ones((5 ,5), "uint8")
blue=cv2.dilate(yellow, kernal)
res=cv2.bitwise_and(img, img, mask = yellow)

#Tracking Colour (Yellow) 
(_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
for pic, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      if(area>300):
           x,y,w,h = cv2.boundingRect(contour)     
           img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)


#Display results
img = cv2.flip(img,1)
image_to_write = cv2.cvtColor(res, cv2.COLOR_RGB2BGR)
cv2.imwrite("Yellow.jpg",res)
image_to_write = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
cv2.imwrite("Color Tracking.jpg",image_to_write)
