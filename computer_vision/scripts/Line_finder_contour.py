#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.bridge = CvBridge()
		self.image = None
		self.cmd = Twist()

		self.top_cutoff = .9
		self.future = 0;



		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		self.image = None

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			pass
		except CvBridgeError, e:
			print e
	
	def on_mouse(self,event,x,y,flag,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			#print self.image[y][x]
			hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
			print hsv[y][x]

	def follow_line(self):
		if self.image != None:
				frame = self.image.copy()
				#cv2.imshow("CAM",frame)	

				
				# Convert BGR to HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

				
				#RED MASKING TAPE HSV
				lower1 = np.array([172,100,150])
				upper1 = np.array([180,255,255])

				mask1 = cv2.inRange(hsv, lower1, upper1)

				lower2 = np.array([-1,100,150])
				upper2 = np.array([5,255,255])
				
				mask2 = cv2.inRange(hsv, lower2, upper2)

				

				mask = cv2.add(mask1,mask2)

				top = mask[:int(len(mask)*self.top_cutoff),:]
				
				bottom = mask[int(len(mask)*self.top_cutoff):,:]

				def is_red(bgr):
					return bgr
					if bgr[2] > 150 and bgr[1]<150 and bgr[0]<150:
						print bgr
						return bgr
					else:
						return [0,0,0]

				#mask = np.array(map(lambda x:map(is_red,x),frame))

				#cv2.imshow("mask",mask)

				t_contours, t_heiarchy = cv2.findContours(top,1,2)
				#print map(lambda x:cv2.contourArea(x),t_contours)

				center =  len(frame[0])/2

				if len(t_contours) > 0:
					t_contour = max(t_contours, key = lambda x:cv2.contourArea(x))
					cv2.drawContours(frame,t_contour,-1,(255,0,0))
					Mt = cv2.moments(t_contour)
					if Mt['m00'] == 0:
						cxt = len(frame[0])/2
						cyt = 0
						#print "WARNING: m00 == 0"
					else:
						cxt = int(Mt['m10']/Mt['m00'])
						cyt = int(Mt['m01']/Mt['m00'])
					cv2.circle(frame,(cxt,cyt),2,(0,255,0))
					self.future = center-cxt

				

				

				contours, heiarchy = cv2.findContours(bottom,1,2)
				

				if len(contours) > 0 and cv2.contourArea(contour):
					contour = max(contours, key = lambda x:cv2.contourArea(x))					
					#print map(lambda x:cv2.contourArea(x),contours)
					print str(cv2.contourArea(contour))
					cv2.drawContours(frame,contour,-1,(255,0,0), offset = (0,int(len(frame)*self.top_cutoff)))

					M = cv2.moments(contour)
					if M['m00'] == 0:
						cx = width = len(frame[0])/2
						cy = 0
						#print "WARNING: m00 == 0"
					else:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])

					cv2.circle(frame,(cx,cy + int(len(frame)*self.top_cutoff)),2,(0,255,0))

					ang_vel = .002 * (center-cx)
					lin_vel = ((2**2) - (ang_vel)**2)**(.5)

					self.cmd = Twist(linear = Vector3(x=.1),angular = Vector3(z=ang_vel))
				else:
					ang_vel = .005* self.future
					self.cmd = Twist(angular = Vector3(z=ang_vel))
				cv2.imshow("CAM",frame)

	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(10)
		while not rospy.is_shutdown():
				self.follow_line()

				self.cmd_vel.publish(self.cmd)
				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	
