#!/usr/bin/env python


import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class D3_Astar():
	#define a 3D space with obstacles

	def __init__(self):
		rospy.init_node('mod_3_star', anonymous=True)
		self.ranges = [0.0,0.0,0.0,0.0,0.0]

		rospy.Subscriber('edrone/range_finder_top',LaserScan,self.obstacle_callback)
		rospy.Subscriber('edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/flag',Bool,self.flag_callback)
		self.next_set = rospy.Publisher('/next_set',NavSatFix,queue_size=10)
		self.position = [0.0,0.0,0.0]
		self.command = NavSatFix()
		self.command.latitude = 19.0
		self.command.longitude = 72.0
		self.command.altitude = 1.26
		self.flag = False
		self.start = (110692.0702932625*(19.0009248718-19.0),-105292.0089353767*(71.9998318945-72.0),27.0)
		self.end = [0.0,0.0,27.0]

	def obstacle_callback(self,msg):
		self.ranges = msg.ranges

	def flag_callback(self,msg):
		self.flag = msg.data
		print(self.flag)

	def gps_callback(self, msg):

		self.position[0] = msg.latitude
		self.position[0] = 110692.0702932625 * (self.position[0] - 19.0)
		self.position[1] = msg.longitude
		self.position[1] = -105292.0089353767 * (self.position[1] - 72.0)
		self.position[2] = msg.altitude
	def has_reached(self,setpoint):
		#print(setpoint,'\n')
		lat = setpoint[0]/110692.0702932625 + 19.0
		lon = setpoint[1]/(-105292.0089353767) +72.0 
		self.command.latitude = float(lat)
		self.command.longitude = float(lon)
		self.command.altitude = float(setpoint[2])
		self.next_set.publish(self.command)

		#if(self.flag):
			#self.flag = False
			#return
	def hueristic(self,start,goal):
		#using euclidean distance, movement in each square in a grid around the object

		dx=abs(start[0]-goal[0])**2
		dy=abs(start[1]-goal[1])**2
		dz=abs(start[2]-goal[2])**2

		return pow(dx+dy+dz,0.5)

	def get_neighbours(self,pos):
		n=[]

		#can move in any cardinal position
		positions=[(0,1,0),(0,-1,0),(1,0,0),(-1,0,0),(1,1,0),(-1,1,0),(-1,-1,0),(1,-1,0),(0,0,1)]
		if self.ranges[0]<5 and self.ranges[0]>1:
			positions.remove((0,1,0))
			positions.remove((1,1,0))
			positions.remove((-1,1,0))
		if self.ranges[1]<5 and self.ranges[1]>1:
			positions.remove((1,0,0))
			positions.remove((1,1,0))
			positions.remove((1,-1,0))
		if self.ranges[2]<5 and self.ranges[2]>1:
			positions.remove((0,-1,0))
			positions.remove((1,-1,0))
			positions.remove((-1,-1,0))
		if self.ranges[3]<5 and self.ranges[3]>1:
			position.remove((-1,0,0))
			position.remove((-1,1,0))
			position.remove((-1,-1,0))
		if self.ranges[4]<5 and self.ranges[4]>1:
			positions.remove((0,0,1))
		
		for dx,dy,dz in positions:
			x2=pos[0]+dx
			y2=pos[1]+dy
			z2=pos[2]+dz

			n.append((x2,y2,z2))

		return n

	def star_search(self):

		closedVertices = set([self.start])
		path=[self.start]
		current=self.start
		c=0
		#print(self.flag)
		while self.end not in path:
			target=[]
			cost=1e10
			print(self.get_neighbours(self.position))
			print('/n')
			for neighbour in self.get_neighbours(self.position):
				if neighbour in closedVertices:
					continue
				if self.hueristic(neighbour,self.end) < cost:
					cost=self.hueristic(neighbour,self.end)
					#print(cost,neighbour)
					target=neighbour
			self.has_reached(target)
			#current=target
			path.append(target)
			closedVertices.add(target)
			#if len(closedVertices)>10: closedVertices.pop()
			#c+=1
			#if c>20:
				#return path

		#raise RuntimeError("Failed")

		return path
	

if __name__=="__main__":
	path=D3_Astar()
	r = rospy.Rate(16.67)
	rospy.Rate(0.50).sleep()
	while not rospy.is_shutdown():
		print(path.position[2])
		if(abs(path.position[2]-27.0)<0.3):
			path.star_search()
		r.sleep()


# a* but sees barrier first then moves acc to best dof
# path is what he just makes when he leaves
# initialy move along line of goal
# if barrier occurs then choose best degree of freedom 
# choose next paths based on set-points




