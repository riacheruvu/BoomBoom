

import cv2
import numpy as np
import imutils

frame = cv2.imread("test_ducks.jpg")

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
cv2.imwrite('frame_blue.jpg',frame)
cv2.imwrite('mask_blue.jpg',mask_blue)
#cv2.imshow('mask_blue', mask_blue)


cv2.imwrite('res_blue.jpg',res)

image = cv2.bitwise_or(frame, frame, mask_blue)
image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
cv2.imwrite('image_color.jpg', image)
hsv_blue[mask_blue > 0] = ([56, 86, 100])
cv2.imwrite('new_hsv_blue.jpg',hsv_blue)

RGB_again = cv2.cvtColor(hsv_blue, cv2.COLOR_HSV2BGR)
gray = cv2.cvtColor(RGB_again, cv2.COLOR_BGR2GRAY)
cv2.imwrite('gray.jpg',gray)

ret, threshold = cv2.threshold(gray, 90, 255, 0)
cv2.imwrite('threshold.jpg',threshold)
cv2.imwrite('ret.jpg',ret)

threshold = cv2.cvtColor(threshold, cv2.COLOR_GRAY2BGR)

added_image = cv2.addWeighted(threshold,0.4,RGB_again,0.1,0)
cv2.imwrite('added_image.jpg',added_image)

def find_marker(image):

	# convert the image to grayscale, blur it, and detect edges

#	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	gray = cv2.GaussianBlur(image, (5, 5), 0)

	edged = cv2.Canny(gray, 35, 125)

	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	cnts = imutils.grab_contours(cnts)
	print("I found {} matching shapes".format(len(cnts)))

	c = max(cnts, key = cv2.contourArea)

	return cv2.minAreaRect(c)


def find_marker_altered(image):

	# convert the image to grayscale, blur it, and detect edges

#	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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

image = cv2.imread("duckie_calibration.jpg")

marker = find_marker(image)

focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
print(marker)
print(marker[1][0])
###############
cnts = find_marker_altered(hsv_blue)

print((cnts)[:5])
cnts = list(sorted(cnts, key = cv2.contourArea, reverse=True))[:8]

for c in cnts:
	# approximate the contour
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
 
	# if our approximated contour has four points, then we
	# can assume that we have found our screen
	if len(approx) == 4:
		pass
		#marker = approx
	else:
		marker = approx	

#for i in range(0, 5):
#	print(i)
	#marker = cv2.minAreaRect(list(sorted(cnts, key = cv2.contourArea, reverse=True))[i])
#	marker = cv2.minAreaRect(list(sorted(cnts, key = cv2.contourArea, reverse=True))[i])

#marker = cv2.minAreaRect(c)

		inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

		#box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)

		#box = np.int0(box)

		#cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
		cv2.drawContours(frame, [marker], -1, (0, 255, 0), 2)

		#cv2.putText(frame, "%.2fft" % (inches / 12),

		#    (frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,

		#    2.0, (0, 255, 0), 3)

image_to_write = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

import os

try: 
	os.remove("newest_edge_detected.jpg")
except:
	pass

cv2.imwrite( "./newest_edge_detected.jpg", image_to_write)

print("Done!")