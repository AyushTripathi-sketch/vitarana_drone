import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from PIL import Image
import numpy as np


class D3_Astar(object):
	#define a 3D space with obstacles

	def __init__(self,barrier,limit):
		self.barriers=barrier  #array containing tuples of barrier positions
		self.limit=limit
		#self.barriers.append()

	def hueristic(self,start,goal):
		#using euclidean distance, movement in each square in a grid around the object

		dx=abs(start[0]-goal[0])**2
		dy=abs(start[1]-goal[1])**2
		dz=abs(start[2]-goal[2])**2

		return round(pow(dx+dy+dz,0.5))

	def get_neighbours(self,pos):
		n=[]

		#can move in any cardinal position
		positions=[(0, 0, 0), (0, 0, -1), (0, 0, 1), (0, -1, 0), (0, -1, -1), (0, -1, 1), (0, 1, 0), (0, 1, -1), (0, 1, 1), (-1, 0, 0), (-1, 0, -1), (-1, 0, 1), (-1, -1, 0), (-1, -1, -1), (-1, -1, 1), (-1, 1, 0), (-1, 1, -1), (-1, 1, 1), (1, 0, 0), (1, 0, -1), (1, 0, 1), (1, -1, 0), (1, -1, -1), (1, -1, 1), (1, 1, 0), (1, 1, -1), (1, 1, 1)]

		for dx,dy,dz in positions:
			x2=pos[0]+dx
			y2=pos[1]+dy
			z2=pos[2]+dz

			if x2<0 or y2<0 or z2<0 or x2>self.limit[0] or y2>self.limit[1] or z2>self.limit[2]:
				continue
			n.append((x2,y2,z2))

		return n

	def moving_cost(self,current,neighbour):
		if neighbour in self.barriers:
			return 1e10 #abysmal cost of going to a obstacle
		return 1 #normal cost


def star_search(start,end,graph):

	G={} #Actual movement cost to each postion from the start
	F={} #Estimated movement cost of start to end going via this position

	#initializing staring values
	G[start]=0
	F[start]=graph.hueristic(start, end)

	closedVertices=set()
	openVertices=set([start])
	camefrom={}

	while len(openVertices)>0:
		#get the vertex in the open list with the lowest F score
		current=None
		currentFscore=None

		for pos in openVertices:
			if current is None or F[pos]<currentFscore:
				currentFscore=F[pos]
				current=pos

		#check if we reached the goal
		if current==end:
			#retrace path
			path=[current]
			while current in camefrom:
				current=camefrom[current]
				path.append(current)
			path.reverse()

			return path, F[end]  #yo baby

		#mark current vertex as closed
		openVertices.remove(current)
		closedVertices.add(current)

		#update scores for vertex surrounding current vertex

		for neighbour in graph.get_neighbours(current):
			if neighbour in closedVertices:
				continue
			candidateG=G[current]+graph.moving_cost(current, neighbour)

			if neighbour not in openVertices:
				openVertices.add(neighbour)
			elif candidateG >=G[neighbour]:
				continue

			#use this G score
			camefrom[neighbour]=current
			G[neighbour]=candidateG
			H=graph.hueristic(neighbour,end)
			F[neighbour]=G[neighbour]+H

	raise RuntimeError("A* failed :(")

 


if __name__=="__main__":
	barriers=[(0, 0, 0), (0, 0, 1), (0, 0, 2), (0, 0, 3), (0, 0, 4), (0, 1, 0), (0, 1, 1), (0, 1, 2), (0, 1, 3), (0, 1, 4), (0, 2, 0), (0, 2, 1), (0, 2, 2), (0, 2, 3), (0, 2, 4), (0, 3, 0), (0, 3, 1), (0, 3, 2), (0, 3, 3), (0, 3, 4), (0, 4, 0), (0, 4, 1), (0, 4, 2), (0, 4, 3), (0, 4, 4), (4, 0, 0), (4, 0, 1), (4, 0, 2), (4, 0, 3), (4, 0, 4), (4, 1, 0), (4, 1, 1), (4, 1, 2), (4, 1, 3), (4, 1, 4), (4, 2, 0), (4, 2, 1), (4, 2, 2), (4, 2, 3), (4, 2, 4), (4, 3, 0), (4, 3, 1), (4, 3, 2), (4, 3, 3), (4, 3, 4), (4, 4, 0), (4, 4, 1), (4, 4, 2), (4, 4, 3), (4, 4, 4), (0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0), (0, 1, 0), (1, 1, 0), (2, 1, 0), (3, 1, 0), (4, 1, 0), (0, 2, 0), (1, 2, 0), (2, 2, 0), (3, 2, 0), (4, 2, 0), (0, 3, 0), (1, 3, 0), (2, 3, 0), (3, 3, 0), (4, 3, 0), (0, 4, 0), (1, 4, 0), (2, 4, 0), (3, 4, 0), (4, 4, 0), (0, 4, 0), (1, 4, 0), (2, 4, 0), (3, 4, 0), (4, 4, 0), (0, 4, 1), (1, 4, 1), (2, 4, 1), (3, 4, 1), (4, 4, 1), (0, 4, 2), (1, 4, 2), (2, 4, 2), (3, 4, 2), (4, 4, 2), (0, 4, 3), (1, 4, 3), (2, 4, 3), (3, 4, 3), (4, 4, 3), (0, 4, 4), (1, 4, 4), (2, 4, 4), (3, 4, 4), (4, 4, 4), (0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0), (0, 0, 1), (1, 0, 1), (2, 0, 1), (3, 0, 1), (4, 0, 1), (0, 0, 2), (1, 0, 2), (2, 0, 2), (3, 0, 2), (4, 0, 2), (0, 0, 3), (1, 0, 3), (2, 0, 3), (3, 0, 3), (4, 0, 3), (0, 0, 4), (1, 0, 4), (2, 0, 4), (3, 0, 4), (4, 0, 4), (0, 0, 4), (1, 0, 4), (2, 0, 4), (3, 0, 4), (4, 0, 4), (0, 1, 4), (1, 1, 4), (2, 1, 4), (3, 1, 4), (4, 1, 4), (0, 2, 4), (1, 2, 4), (3, 2, 4), (4, 2, 4), (0, 3, 4), (1, 3, 4), (2, 3, 4), (3, 3, 4), (4, 3, 4), (0, 4, 4), (1, 4, 4), (2, 4, 4), (3, 4, 4), (4, 4, 4), (6, 0, 0), (6, 0, 1), (6, 0, 2), (6, 0, 3), (6, 0, 4), (6, 0, 5), (6, 0, 6), (6, 0, 7), (6, 1, 0), (6, 1, 1), (6, 1, 2), (6, 1, 3), (6, 1, 4), (6, 1, 5), (6, 1, 6), (6, 2, 0), (6, 2, 1), (6, 2, 2), (6, 2, 3), (6, 2, 4), (6, 2, 5), (6, 2, 6), (6, 2, 7), (6, 3, 0), (6, 3, 1), (6, 3, 2), (6, 3, 3), (6, 3, 4), (6, 3, 5), (6, 3, 6), (6, 3, 7), (6, 4, 0), (6, 4, 1), (6, 4, 2), (6, 4, 3), (6, 4, 4), (6, 4, 5), (6, 4, 6), (6, 4, 7), (6, 5, 0), (6, 5, 1), (6, 5, 2), (6, 5, 3), (6, 5, 4), (6, 5, 5), (6, 5, 6), (6, 5, 7), (6, 6, 0), (6, 6, 1), (6, 6, 2), (6, 6, 3), (6, 6, 4), (6, 6, 5), (6, 6, 6), (6, 6, 7), (6, 7, 0), (6, 7, 1), (6, 7, 2), (6, 7, 3), (6, 7, 4), (6, 7, 5), (6, 7, 6), (6, 7, 7)]
	limit=(7,7,7)
	graph=D3_Astar(barriers, limit)

	result,cost=star_search((0,0,0), (7,7,7), graph)

	print("path ",result)
	print("cost ",cost)

	ax = plt.axes(projection='3d')
	rx=np.array([v[0] for v in result])
	ry=np.array([v[1] for v in result])
	rz=np.array([v[2] for v in result])
	ax.plot3D(rx,ry,rz)

	bx=np.array([v[0] for v in barriers])
	by=np.array([v[1] for v in barriers])
	bz=np.array([v[2] for v in barriers])

	ax.scatter3D(bx,by,bz,color ='red')

	plt.show()





