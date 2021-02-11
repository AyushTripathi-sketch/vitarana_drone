#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from vitarana_drone.msg import *
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import cv2
from sys import argv
import zbar
from std_msgs.msg import Float32
import numpy as np
import rospy
import math

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test', anonymous=True, disable_signals=False) #Initialise rosnode

		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera
		rospy.Subscriber("/marker_pub",Int32,self.marker_callback)
		#rospy.Subscriber("/Z_diff",Float32,self.Z_callback)#
		#rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.obstacle_callback)
		self.error_pub = rospy.Publisher('/edrone/marker_data',MarkerData,queue_size=10) #Publishing the MarkerData
		self.recieved = False
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		#self.Z = [0.0]
		
		self.msg = MarkerData()
		self.msg.marker_id = 0
		self.msg.err_x_m = 0.0
		self.msg.err_y_m = 0.0

		img_width = 400
		hfov_rad = 1.3962634  #horizontal field of view
		self.focal_length = (img_width/2)/math.tan(hfov_rad/2)
	# Callback function of amera topic
	def image_callback(self, data):
		#print("callback")
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			# cv2.imshow('img', self.img)
			self.recieved = True
		except CvBridgeError as e:
			print(e)
			return

	def marker_callback(self,msg):
		self.msg.marker_id = msg.data
	#def obstacle_callback(self,msg):
	#	self.Z = msg.ranges

	#def Z_callback(self,msg):
	#	self.Z = msg.data
	#	self.Z = float(self.Z) 
	def detect_marker(self):
		if self.recieved:
			logo_cascade = cv2.CascadeClassifier('/home/maut/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
			img = self.img
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

			# image, reject levels level weights.
			logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.1,minNeighbors=3)

			for (x, y, w, h) in logo:
				cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
				print(x+w/2,y+h/2)
				self.msg.err_x_m = ((x+float(w)/2-200)*(20.0)/self.focal_length) #calculating the x_error from drone to marker
				self.msg.err_y_m = ((y+float(h)/2-200)*(20.0)/self.focal_length) #calculating the y_error from drone to marker
				print(self.msg)
				#print(self.Z)
			cv2.imshow('img', img)#display the detected image 
			cv2.waitKey(2)#wait for 2 millisecond before closing the output
			self.error_pub.publish(self.msg)#publish the Marker Data

if __name__ == '__main__':
	img = image_proc()
	r = rospy.Rate(10.0) # Rate at which the node runs
	while not rospy.is_shutdown():
		img.detect_marker()
		r.sleep()

    
