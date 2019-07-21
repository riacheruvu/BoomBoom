#Citation to come!
import cv2
import numpy as np


frame = cv2.imread("duckies.jpg")

# Convert BGR to HSV
hsv_yellow = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# define range of yellow color in HSV
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])

# Threshold the HSV image to get only yellow colors
mask_yellow = cv2.inRange(hsv_yellow, lower_yellow, upper_yellow)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(frame, frame, mask_yellow)

res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
cv2.imwrite('frame_yellow.jpg',frame)
cv2.imwrite('frame_yellow.jpg',mask_yellow)
cv2.imwrite('frame_yellow.jpg',res)

cv2.destroyAllWindows()
