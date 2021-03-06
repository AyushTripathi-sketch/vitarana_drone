#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_control which controls the latitude, longitude and altitude of the eDrone.
This node publishes and subsribes the following topics:
		PUBLICATIONS                 SUBSCRIPTIONS
		/edrone/drone_command        /edrone/gps
		/throttle_error              /pid_tuning_altitude
		/zero_error					 /pid_tuning_roll
		/lat_error					 /pid_tuning_pitch
		/lon_error
		/reached

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
import rospy
import time
from std_msgs.msg import Bool
import rosservice
import pandas as pd
class PositionControl():
	"""docstring for PositionControl"""
	def __init__(self):
		rospy.init_node('position_controller', anonymous=True) # initializing ros node with name position_controller

		#This correspond to the position coordinates of the point the edrone has to reach
		#[lat, lon, alt]
		#read the csv file having destination coordinates
		col_names = ['Cell_no','Latitude','Longitude','Altitude']
		df = pd.read_csv('/home/maut/catkin_ws/src/vitarana_drone/scripts/manifest2.csv',names=col_names,header=None)
		self.lat = df.loc[:,'Latitude']
		self.lon = df.loc[:,'Longitude']
		self.alt = df.loc[:,'Altitude']

		self.safe_alt =14.0
		self.setpoint_set = [[19.0,72.0,14.0],[19.0,71.99995726171,14.0],[19.0,71.99995726171,8.40999749139],
						   [19.0,71.99995726171,self.alt[2]+12.0],[self.lat[2],self.lon[2],self.alt[2]+12.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
						   [19.0,72.0,self.alt[0]+12.0],[19.0,72.0,20.0],[19.0,72.0,8.43999749139],[19.0000135511,71.9999430161,self.alt[1]+12.0],
						   [self.lat[1],self.lon[1],self.alt[1]+12.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[19.0,71.99995726171,self.alt[1]+12.0],[19.0,71.99995726171,self.alt[1]+12.0],[19.0,71.99995726171,8.44099749139],
						   [19.0,71.99995726171,self.alt[2]+12.0],[19.000332997364,71.99995726171,self.alt[2]+12.0],[19.000332997364,71.99995726171,self.alt[2]+27.0],[self.lat[2],self.lon[2],self.alt[2]+12.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[19.0,72.0,self.alt[2]+12.0],
						   [19.0,72.0,20.0],[19.0,72.0,8.44],[19.0,72.0,8.44]]
		
		self.setpoint_des = [0.0,0.0,0.0]
		self.setpoint = [19.0,72.0,self.safe_alt]
		#This corresponds to the current position coordinates of the edrone
		#[lat, lon, alt]
		self.position = [19.0,72.0,0.31]

		
		#setting of Kp, Ki and Kd for [alt, lon, lat]
		self.Kp = [7.68,2.04,2.04]
		self.Ki = [0.0,0.0,0.0]
		self.Kd = [100.2,100.5,100.5]

		#counter variable to count the time the edrone has been on a specific target setpoint for
		self.count = self.chk_no = self.flag = 0
		self.chk_no = 1
		#flag to ascertain if the edrone has reached its final position
		self.has_reached = False

		# min and max values to be sent for [rcRoll, rcPitch, rcThrottle]
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.closedVertices = set([])
		# variables to store the differential error and sum of errors for pid tuning
		self.prev_error = [0.0,0.0,0.0]
		self.e_sum = [0.0,0.0,0.0]
		self.err_trgt = [0.0,0.0,0.0]
		self.err_x = 0.0 
		self.err_y = 0.0
		self.ranges = [0.0,0.0,0.0,0.0,0.0]

		# Declaring command of message type edrone_cmd and initializing the values
		self.command = edrone_cmd()
		self.command.rcRoll = 1500.0
		self.command.rcPitch = 1500.0
		self.command.rcYaw = 1500.0
		self.command.rcThrottle = 1000.0

		# This is the sample time in which pid is run
		self.sample_time = 0.060

		# Initializing the values to be sent for [rcPitch, rcRoll, rcThrottle]
		self.out_lat = 1500
		self.out_lon = 1500
		self.out_alt = 1000

		# pubishing '/drone_command', 'throttle_error', 'lat_error', 'lon_error', '/reached'
		self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=10)
		self.throttle_pub = rospy.Publisher('throttle_error', Float64, queue_size=10)
		self.lat_pub = rospy.Publisher('lat_error',Float64,queue_size=10)
		self.lon_pub = rospy.Publisher('lon_error',Float64,queue_size=10)
		#self.Z_pub = rospy.Publisher('/Z_diff',Float64,queue_size=10)		

		# Subscribing 'edrone/gps', 'pid_tuning_altitude', '/pid_tuning_roll', '/pid_tuning_pitch'
		rospy.Subscriber('edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/marker_data',MarkerData,self.marker_callback)
		rospy.Subscriber('pid_tuning_altitude', PidTune, self.throttle_pid_tune)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('edrone/range_finder_top',LaserScan,self.obstacle_callback)
		#rospy.Subscriber('/setpoint_dec', NavSatFix, self.set_position)
		#Defining the Gripper Service
		self.service_name  = "/edrone/activate_gripper"
		self.srv_class = rosservice.get_service_class_by_name(self.service_name)
		
			

	# function to check if error between the current position coordinates and setpoint is low enough to be ignored
	# it is called from the pid() function
	def check_error(self, error):
		if(self.flag==1):
			if abs(error[0])<1 and abs(error[1])<1 and abs(error[2])<0.08:
				return True
			else:
				return True
		elif(self.flag==2):
			if abs(error[0])<1 and abs(error[1])<1 and abs(error[2])<0.08:
				return True
			else:
				return False
		else:
			if abs(error[0])<0.05 and abs(error[1])<0.05 and abs(error[2])<0.08:
				return True
			else:
				return False


	# gps callback function 
	# this function gets executed each time when gps publishes to 'edrone/gps'
	def gps_callback(self, msg):

		self.position[0] = msg.latitude
		self.position[1] = msg.longitude
		self.position[2] = msg.altitude

	def throttle_pid_tune(self, msg):

		self.Kp[0] = msg.Kp*0.06
		self.Ki[0] = msg.Ki*0.008
		self.Kd[0] = msg.Kd*0.3

	# Callback functions for /pid_tuning_roll and /pid_tuning_pitch respectively
	# This function gets executed each time when /tune_pid publishes /pid_tuning_roll and /pid_tuning_pitch respectively
	# when tuning here in this node, the callback function in attitude_controller must be commented out so as to not cause anomalies
	def roll_set_pid(self,msg):
		self.Kp[1] = msg.Kp * 0.06
		self.Ki[1] = msg.Ki * 0.008
		self.Kd[1] = msg.Kd * 0.3
 
	def pitch_set_pid(self, msg):
		self.Kp[2] = msg.Kp * 0.06
		self.Ki[2] = msg.Ki * 0.008
		self.Kd[2] = msg.Kd * 0.3

	def set_position(self, msg):
		self.setpoint_des[0] = msg.latitude
		self.setpoint_des[1] = msg.longitude
		self.setpoint_des[2] = msg.altitude

	def obstacle_callback(self,msg):
		self.ranges = msg.ranges

	def marker_callback(self,msg):
		self.marker_id = msg.marker_id
		self.err_x = msg.err_x_m/110692.0702932625
		self.err_y = msg.err_y_m/(-105292.0089353767)

	def hueristic(self,start,goal):
		#using euclidean distance, movement in each square in a grid around the object

		dx=abs(start[0]-goal[0])**2
		dy=abs(start[1]-goal[1])**2
		dz=abs(start[2]-goal[2])**2

		return pow(dx+dy+dz,0.5)

	def get_neighbours(self,pos):
		n=[]

		if all(i>20.0 for i in self.ranges):
			#can move in any cardinal position
			positions=[(0,10,0),(0,-10,0),(10,0,0),(-10,0,0),(8,8,0),(-8,8,0),(-8,-8,0),(8,-8,0),(0,0,10)]
			for dx,dy,dz in positions:
				x2=pos[0]+dx
				y2=pos[1]+dy
				z2=pos[2]+dz

				n.append((x2,y2,z2))

			return n
		else:
			#can move in any cardinal position
			self.flag=2
			positions=[(0,8,0),(0,-8,0),(8,0,0),(-8,0,0),(4,4,0),(-4,4,0),(-4,-4,0),(4,-4,0),(0,0,8)]
			if self.ranges[0]<10 and self.ranges[0]>1:
				if (0,8,0) in positions: positions.remove((0,8,0))
				if (4,4,0) in positions: positions.remove((4,4,0))
				if (-4,4,0) in positions: positions.remove((-4,4,0))
			if self.ranges[1]<10 and self.ranges[1]>1:
				if (4,4,0) in positions: positions.remove((4,4,0))
				if (0,8,0) in positions: positions.remove((0,8,0))
				if (8,0,0) in positions: positions.remove((8,0,0))
				if (4,-4,0) in positions: positions.remove((4,-4,0))
			if self.ranges[2]<10 and self.ranges[2]>1:
				if (0,-8,0) in positions: positions.remove((0,-8,0))
				if (4,-4,0) in positions: positions.remove((4,-4,0))
				if (-4,-4,0) in positions: positions.remove((-4,-4,0))
			if self.ranges[3]<10 and self.ranges[3]>1:
				if (-8,0,0) in positions: positions.remove((-8,0,0))
				if (-4,4,0) in positions: positions.remove((-4,4,0))
				if (-4,-4,0) in positions: positions.remove((-4,-4,0))
			if self.ranges[4]<10 and self.ranges[4]>1:
				if (0,0,8) in positions: positions.remove((0,0,8))
			
			for dx,dy,dz in positions:
				x2=pos[0]+dx
				y2=pos[1]+dy
				z2=pos[2]+dz

				n.append((x2,y2,z2))

			return n
		return n

	def star_search(self,start,end):

		waypint=[0.0,0.0,0.0]
		min_cost=1e10
		self.closedVertices.add(tuple(start))
		for neighbour in self.get_neighbours(start):
			if neighbour in self.closedVertices:
				continue
			cost=self.hueristic(neighbour,end)
			if cost < min_cost:
				min_cost=cost
				waypoint=neighbour
		#closedVertices.add(waypoint)
		waypint[0]=19.0 + waypoint[0]/110692.0702932625
		waypint[1]=72.0 + waypoint[1]/-105292.0089353767
		waypint[2]=waypoint[2]
		return waypint

	def pid(self):

		# calculating the error in lat, lon and alt and rescaling the error in lat, lon so that it corresponds to roughly in metres
		error = [(x1-x2) for (x1,x2) in zip(self.setpoint, self.position)]
		error[0] = error[0] * 60000#111000
		error[1] = error[1] * 60000#111000
		#print("\n")
		#print(error[2])
		#print("\n")
		#if(self.setpoint_des[0]!=0 and self.flag==0 and self.check_error(error)):
			#self.setpoint=[19.0007046575,71.9998955286,22.1599967919]
			#self.flag==1
			#self.setpoint_set.append(self.setpoint_des)
			
		# here, it is checked if the edrone has reached the setpoint and been stable for some time
		# also the setpoint is changed to the next setpoint if the edrone has reached the required point
		# if the edrone has reached the final setpoint, the has_reached flag is set to True
		if self.check_error(error):
			self.count += 1
		 	#print(self.count)
		 	if self.chk_no==29:
		 		self.has_reached=True
		 	if self.count>=10:
				self.count = 0
				self.flag = 0
		 		self.target = self.setpoint_set[self.chk_no]
		 		if abs(self.target[0]-self.position[0])*111000 < 0.5 and abs(self.target[1]-self.position[1])*111000 < 0.5 :
		 			self.chk_no += 1
		 			print(self.chk_no)
		 			self.target = self.setpoint_set[self.chk_no]
		 		if (self.chk_no == 5 or self.chk_no == 13 or self.chk_no == 22):# and (abs(self.position[0]-self.target_set[self.chk_no][0])<1):
	 				#self.chk_no+=1
	 				self.setpoint[0] = self.position[0] + self.err_x
	 				self.setpoint[1] = self.position[1] - self.err_y
	 				self.setpoint[2] = self.setpoint_set[self.chk_no-1][2]
	 				self.setpoint_set[self.chk_no] = [self.setpoint[0],self.setpoint[1],self.setpoint[2]]
	 				self.setpoint_set[self.chk_no+1] = [self.setpoint[0], self.setpoint[1], self.setpoint_set[self.chk_no-1][2]-11.0]
	 				self.setpoint_set[self.chk_no+2] = [self.setpoint[0], self.setpoint[1], self.setpoint_set[self.chk_no-1][2]]
		 		else:
		 			if abs(self.target[0]-self.position[0])*111000>15 or abs(self.target[1]-self.position[1])*111000>15:
		 				start=[0.0,0.0,0.0]
		 				end=[0.0,0.0,0.0]
		 				self.flag=1
		 				start[0]= 110692.0702932625 * (self.position[0] - 19.0)
		 				start[1]=-105292.0089353767 * (self.position[1] - 72.0)
		 				start[2]=self.position[2]
		 				end[0]= 110692.0702932625 * (self.target[0] - 19.0)
		 				end[1]=-105292.0089353767 * (self.target[1] - 72.0)
		 				end[2]=self.target[2]
		 				print("working")
		 				self.setpoint=self.star_search(start,end)
		 			else:
		 				self.setpoint=self.target
		 				self.closedVertices = set([])
		 		print(self.setpoint)

		#rospy.loginfo(resp)
		if(self.chk_no==3 or self.chk_no==11 or self.chk_no==19):
			while(True):
				req = self.srv_class._request_class(activate_gripper=True)
				#if needed set any arguments here
				#rospy.loginfo(req)
				rospy.wait_for_service(self.service_name)
				srv_client = rospy.ServiceProxy(self.service_name, self.srv_class)
				resp = srv_client(req)
				if resp.result:
					print("package received")
					break

		if(self.chk_no==7 or self.chk_no==15 or self.chk_no==23):
			while(True):
				req = self.srv_class._request_class(activate_gripper=False)
				rospy.wait_for_service(self.service_name)
				srv_client = rospy.ServiceProxy(self.service_name, self.srv_class)
				resp = srv_client(req)
				if resp.result==False:
					print("package dropped")
					break
				continue			
		# else:
		# 	self.count = 0
		d_error = [x1-x2 for (x1,x2) in zip(error, self.prev_error)]

		# calculation of error sum
		self.e_sum = [x1+x2 for (x1,x2) in zip(error, self.e_sum)]
		#print(self.e_sum)
		##print('/n')
		# Calculation of PID output required for latitude, longitude and altitude respectively
		self.out_lat = float(1500 + (error[0]*self.Kp[2] + self.e_sum[0]*self.Ki[2] + d_error[0]*self.Kd[2]))
		self.out_lon = float(1500 + (error[1]*self.Kp[1] + self.e_sum[1]*self.Ki[1] + d_error[1]*self.Kd[1]))
		self.out_alt = float(1500 + (error[2]*self.Kp[0] + self.e_sum[2]*self.Ki[0] + d_error[2]*self.Kd[0])*50)

		# Checking if any of the output values surpass the min or max values and resetting them if they do
		if self.out_lat > self.max_values[0]:
			self.out_lat = self.max_values[0]

		if self.out_lat < self.min_values[0]:
			self.out_lat = self.min_values[0]

		if self.out_lon > self.max_values[1]:
			self.out_lon = self.max_values[1]

		if self.out_lon < self.min_values[1]:
			self.out_lon = self.min_values[1]

		if self.out_alt > self.max_values[0]:
			self.out_alt = self.max_values[0]

		if self.out_alt < self.min_values[0]:
			self.out_alt = self.min_values[0]

		# Updating the value of the previous error
		self.prev_error = error

		# Updating the message contents to be published to '/drone_command'
		self.command.rcRoll = self.out_lon
		self.command.rcPitch = self.out_lat
		self.command.rcThrottle = self.out_alt

		#Setting the output values to zero if the edrone has reached its final position
		if self.has_reached:
			self.command.rcRoll = 1500.0
			self.command.rcPitch = 1500.0
			self.command.rcThrottle = 1000.0

		# publishing the required values
		self.cmd_pub.publish(self.command)
		self.throttle_pub.publish(error[2])
		self.lat_pub.publish(error[0])
		self.lon_pub.publish(error[1])


if __name__ == '__main__':
	
	# creating an object of PositionControl class
	pos_controller = PositionControl()
	r = rospy.Rate(1/pos_controller.sample_time) # Rate takes value in hertz

	# time to be checked to create an additional delay to account for the opening of gazebo
	t0 = rospy.Time.now().to_sec()

	while not rospy.is_shutdown():
		# initial delay of 6 seconds
		t1 = rospy.Time.now().to_sec()
		if t1 - t0 <3:
			continue

		# running the pid
		pos_controller.pid()

		# Checking if the edrone has reached its final postion and publishing that it has if it has
		#if pos_controller.has_reached:
			#pos_controller.reached_pub.publish(pos_controller.has_reached)
			#break

		r.sleep()
