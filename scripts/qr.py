#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sys import argv
import zbar
import numpy as np
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test', anonymous=True, disable_signals=False) #Initialise rosnode 
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.new_pub = rospy.Publisher('/setpoint_dec', NavSatFix, queue_size=10)
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.command = NavSatFix()
		self.command.latitude = 19.0
		self.command.longitude = 72.0
		self.command.altitude = 1.26

	# Callback function of amera topic
	def image_callback(self, data):
		print("callback")
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.decode()
		except CvBridgeError as e:
			print(e)
			return

	# Find barcodes and QR codes
	def decode(self):
		
		# Create zbar scanner
		scanner = zbar.ImageScanner()
		
		# Configure scanner
		scanner.parse_config('enable')	

		# Convert image to grayscale
		imGray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# Find height and width
		height, width = imGray.shape

		# Get raw image bytes
		raw = imGray.tobytes()

		# Wrap image data in a zbar image
		self.decodedObjects = zbar.Image(width, height, 'Y800', raw)

		# Scan the image for barcodes and QRCodes
		scanner.scan(self.decodedObjects)

		setpoint = ["", "", ""]

		for self.decodedObject in self.decodedObjects:
			string = self.decodedObject.data
			i=0
			for char in string:
				if(char==','):
					i += 1
				else:
					setpoint[i] += char

			print(setpoint)
			self.command.latitude = float(setpoint[0])
			self.command.longitude = float(setpoint[1])
			self.command.altitude = float(setpoint[2])
			self.new_pub.publish(self.command)
			rospy.Rate(0.01).sleep()
			rospy.signal_shutdown("Location Scanned and Published")

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
