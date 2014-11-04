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
		#self.image = None
		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		self.image = cv2.imread("StopSign16in.png")

	def update_image(self,msg):
		try:
			#self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			#cv2.imwrite("capture.png", self.image)
			self.image = cv2.imread("StopSign16in.png")
			pass
		except CvBridgeError, e:
			print e
	
	def on_mouse(self,event,x,y,flag,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
			print hsv[y][x]


	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.image != None:
				frame = self.image
				#cv2.imshow("CAM",frame)	

				# Convert BGR to HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

				# define range of blue color in HSV
				lower = np.array([30,135,150])
				upper = np.array([40,255,255])

				#Green tape range [35,50,0] to [50,255,255]

				mask = cv2.inRange(hsv, lower, upper)

				contours, heiarchy = cv2.findContours(mask,1,2)

				contour = max(contours, lambda x:cv2.contourArea(x))
				#print type(contour) + " " + type(contours[0])
				print map(lambda x:cv2.contourArea(x),contours)
				#print str(cv2.contourArea(contour))
				cv2.drawContours(frame,contour,-1,(255,0,0))


				# RED STOP SIGN
				lower = np.array([0,230,90])
				upper = np.array([12,255,150])

				#Green tape range [35,50,0] to [50,255,255]

				# Threshold the HSV image to get only blue colors
				mask = cv2.inRange(hsv, lower, upper)

				contours, heiarchy = cv2.findContours(mask,1,2)
				contour = contours[0] 
				#cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
				cv2.drawContours(frame,contour,-1,(0,255,0))

				#create the rectangle around the stop sign
				x,y,w,h = cv2.boundingRect(contour)
				box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
				#extract rectangle width
				width = w

				#draw Rectangle
				cv2.drawContours(frame,[box],0,(0,0,255),2)

				print "width:",width

				cv2.imshow("CAM",frame)
				cv2.imshow("mask",mask)



				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	
