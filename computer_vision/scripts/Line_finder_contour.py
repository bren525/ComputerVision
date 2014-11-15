#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		rospy.odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.bridge = CvBridge()
		self.image = None
		self.cmd = Twist()
		self.stop = False

		self.top_cutoff = .9
		self.future = 0;
		self.odom = (0,0)
		self.odom_zero = None
		
		self.image = None

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			pass
		except CvBridgeError, e:
			print e
	def update_odom(self,msg):
		self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
		print "odom update:", self.odom

	def on_mouse(self,event,x,y,flag,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			#print self.image[y][x]
			hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
			print hsv[y][x] #Super useful for color calibrating

	def follow_line(self):
		'''Main logic that guides line following and stopping at teh stop sign'''
		if self.image != None:
				frame = self.image.copy()
				#cv2.imshow("CAM",frame)	

				# Convert BGR to HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
				
				#RED MASKING TAPE HSV
				lower1 = np.array([172,100,100])
				upper1 = np.array([180,255,255])

				mask1 = cv2.inRange(hsv, lower1, upper1)

				lower2 = np.array([-1,100,100])
				upper2 = np.array([13,255,255])
				
				mask2 = cv2.inRange(hsv, lower2, upper2)
				
				#Combine the two red filtering masks because RED wraps the hue space (172 to 13)
				mask = cv2.add(mask1,mask2)

				# GREEN STOP SIGN Threshholds in HSV
				stop_lower = np.array([30,130,120])
				stop_upper = np.array([75,200,210])
				#Green Stop Sign Mask
				stop_mask = cv2.inRange(hsv, stop_lower, stop_upper)
				stop_contours, stop_heiarchy = cv2.findContours(stop_mask,1,2)
				
				#if the stop command has already been sent, check to see if we've travelled far enough to stop
				if(self.odom_zero != None):
					dist = ((self.odom_zero[0][0] - self.odom[0])**2 + (self.odom_zero[0][1] - self.odom[1])**2)**(.5) 
					print dist, self.odom_zero[1]
					if dist >= self.odom_zero[1]:
						print "stop command sent"
						self.cmd = Twist()
						self.stop = True

				if(len(stop_contours) > 0 and self.stop != True):
					#we assume the largest contour by area is the one we want
					stop_contour = max(stop_contours, key = lambda x:cv2.contourArea(x))
					#define the rectangle around contour
					x,y,w,h = cv2.boundingRect(stop_contour)
					box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
					#Draw Rectangle
					cv2.drawContours(frame,[box],0,(0,0,255),2)


					#Calculate the Distance in inches
					distance_in = (w - 528.0)/-15.0
					#oops lets convert to meters for odom
					distance_m = distance_in*0.0254
					
					#if we're a certain distance away, warn the odom
					if(abs(distance_m - 0.4218)< 0.0254):
						print "self.odom:", self.odom
						self.odom_zero = (self.odom, .15*distance_m)
						print distance_m
						print "stopping..."

				#Only look at bottom 10% of image
				bottom = mask[int(len(mask)*self.top_cutoff):,:]
				
				#Grab the contours of our image
				contours, heiarchy = cv2.findContours(bottom,1,2)

				if len(contours) > 0  and self.stop != True:
					contour = max(contours, key = lambda x:cv2.contourArea(x)) #Select the largest contour
					if cv2.contourArea(contour) > 1000: #Make sure the largest contour is somewhat large
						cv2.drawContours(frame,contour,-1,(255,0,0), offset = (0,int(len(frame)*self.top_cutoff)))
						
						#Find centroid of contour
						M = cv2.moments(contour)
						if M['m00'] == 0:
							cx = width = len(frame[0])/2
							cy = 0
							#print "WARNING: m00 == 0"
						else:
							cx = int(M['m10']/M['m00'])
							cy = int(M['m01']/M['m00'])

						cv2.circle(frame,(cx,cy + int(len(frame)*self.top_cutoff)),2,(0,255,0))
						
						#Set the future offset in case we lose sight of line
						self.future = center-cx
						
						#Tuned proportionality constant for staying on the line
						ang_vel = .003 * (center-cx)
						
						#Maximum linear velocity
						lin_vel = .5

						
						self.cmd = Twist(linear = Vector3(x=lin_vel),angular = Vector3(z=ang_vel))

				elif self.stop != True:
					if self.future != 0:
						#Assume we're making a tight turn, turn as fast as possible!
						ang_vel = 2.5 * (self.future/abs(self.future))
						self.cmd = Twist(linear = Vector3(x=.5),angular = Vector3(z=ang_vel))
					else:
						#We've reached a dead end
						self.cmd = Twist()
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

	
