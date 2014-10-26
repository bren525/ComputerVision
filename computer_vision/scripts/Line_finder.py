#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.bridge = CvBridge()
		self.image = cv2.imread("test.png")

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError, e:
			print e
	
	def run(self):
		r=rospy.Rate(1)
		while not rospy.is_shutdown():

			frame = self.image
			cv2.imshow("CAM",frame)

			edges = cv2.Canny(frame,50,50)		
			cv2.imshow("EDGES",edges)

			# Convert BGR to HSV
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# define range of blue color in HSV
			lower = np.array([35,200,80])
			upper = np.array([55,255,255])

			#Green tape range [35,50,0] to [50,255,255]

			# Threshold the HSV image to get only blue colors
			mask = cv2.inRange(hsv, lower, upper)

			# Bitwise-AND mask and original image
			res = cv2.bitwise_and(frame,frame, mask= mask)

			cv2.imshow("Filtered",res)

			edges = cv2.Canny(res,200,200)		
			cv2.imshow("EDGES",edges)
			cv2.waitKey(50)
			r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	
