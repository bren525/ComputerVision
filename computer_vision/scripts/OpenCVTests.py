#!/usr/bin/env python

import cv2
import numpy as np

if __name__ == '__main__':
	cap = cv2.VideoCapture(0)


	while True:
		ret, frame = cap.read()
		cv2.imshow("WEBCAM",frame)

		edges = cv2.Canny(frame,50,50)		
		cv2.imshow("EDGES",edges)

		# Convert BGR to HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# define range of blue color in HSV
		lower = np.array([35,50,75])
		upper = np.array([50,255,255])

		#Green tape range [35,50,0] to [50,255,255]

		# Threshold the HSV image to get only blue colors
		mask = cv2.inRange(hsv, lower, upper)

		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(frame,frame, mask= mask)

		cv2.imshow("Filtered",res)

		edges = cv2.Canny(res,200,200)		
		cv2.imshow("EDGES",edges)

		cv2.waitKey(50)
