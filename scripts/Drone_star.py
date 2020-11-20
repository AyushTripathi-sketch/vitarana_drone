#!/usr/bin/env python

from vitarana_drone.msg import *
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import rospy
import time
import numpy as np
import pandas as pd

class MotionPlanning:
	def __init__(self):

		self.position = [19.0,72.0,0.31]
		self.target_position = np.array([0.0,0.0,0.0])
		self.waypoints = []
		self.in_mission = True
		self.check_state = {}
		self.target_altitude = 5
		self.safety_distance = 5

		rospy.Subscriber('edrone/gps', NavSatFix, self.gps_callback)

	# gps callback function 
	# this function gets executed each time when gps publishes to 'edrone/gps'
	def gps_callback(self, msg):

		self.position[0] = msg.latitude
		self.position[1] = msg.longitude
		self.position[2] = msg.altitude

	def read_csv(self):
		lat0_lon0_alt0 = pd.read_csv('colliders.csv', nrows = 1, header = None)
		lat0,lon0, alt0 = lat0_lon0_alt0.iloc[0,0], lat0_lon0_alt0.iloc[0,1], lat0_lon0_alt0.iloc[0,2] 
		_, lat0 = lat0.split()
		_, lon0 = lon0.split()
		lat0 = np.float32(lat0)
		lon0 = np.float32(lon0)
		alt0 = np.float32(alt0)

		#set home position
		self.home_position = [lon0,lat0,alt0]

		#local position w.r.t home position
		self.current_local_pos = [(x1-x2) for (x1,x2) in zip(self.position,self.home_position)]

		data = np.loadtxt('colliders.csv',delimeter=',', dtype='Float32', skiprows = 3)
		grid, north_offset, east_offset = create_grid(data, self.target_altitude, self.safety_distance)

	def create_grid(self):
		
