#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		#rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.bridge = CvBridge()
		self.image = None
		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		#self.image = cv2.imread("test.png")

	def update_image(self,msg):
		try:
			#self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			self.image=open("NormalView.png")
			pass
		except CvBridgeError, e:
			print e

	def two_cluster_lines(self, data):
		pass
	
	def run(self):
		r=rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.image != None:
				frame = self.image
				#cv2.imshow("CAM",frame)	

				# Convert BGR to HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

				# define range of blue color in HSV
				lower = np.array([30,135,150])
				upper = np.array([55,255,255])

				#Green tape range [35,50,0] to [50,255,255]

				# Threshold the HSV image to get only blue colors
				mask = cv2.inRange(hsv, lower, upper)

				# Bitwise-AND mask and original image
				res = cv2.bitwise_and(frame,frame, mask= mask)

				

				gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

				cv2.imshow("Filtered",gray)

				edges = cv2.Canny(gray,200,200, apertureSize = 3)
				
				lines = cv2.HoughLines(edges,1,np.pi/180,60)
				print lines
				#holderList = []
				#line1 = []
				#line2 = []
				#theta1 = 0
				#theta2 = 0
				if lines != None:
					#for rho,theta in lines[0]:
					#	holderList.append(theta)
					#	diff = abs(holderList[0] - theta)
					#	if diff < .2:
					#		line1.append([rho,theta])
					#	else:
					#		line2.append([rho,theta])
					#for rho,theta in line1:
					#	theta1 += theta
					#for rho,theta in line2:
					#	theta2 += theta
					#line1avg = theta1/len(line1)
					#line2avg = theta2/len(line2)
					#lines = [line1avg,line2avg]
					for theta in lines:
						a = np.cos(theta)
						b = np.sin(theta)
						x0 = a*rho
						y0 = b*rho
						x1 = int(x0 + 1000*(-b))
						y1 = int(y0 + 1000*(a))
						x2 = int(x0 - 1000*(-b))
						y2 = int(y0 - 1000*(a))
						cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
				
				cv2.imshow("CAM",frame)
				cv2.imshow("EDGES",edges)
				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	
