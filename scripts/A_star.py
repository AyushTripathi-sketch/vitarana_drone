import numpy as np

class Node:

	'''A* is the path finding algorothim
	   parent is the parent node of the current node
	   position is the the position of the current node in the Maze
	   g is the cost from start to the current node
	   h is the heuristic based estimated cost of the current node from the end node
	   f is the total cost of the current node i.e. f=g+h'''

	def __init__(self,parent=None,position=None):
		self.parent = parent
		self.position = position

		self.g=self.h=self.f=0

	def __eq__(self,other):
		return self.position == other.position

def return_path(current_node,maze):
	path=[]
	no_rows,no_columns,no_layers = np.shape(maze)
	# here we create initialized result with -i in every position
	result = [[[-1 for k in range(no_layers)] for j in range(no_columns)] for i in range(no_columns)]
	current = current_node
	while current is not None:
		path.append(current.position)
		current = current.parent
	# Return reversed path as we need to show from startto end path
	path = path[:-1]
	start_value = 0
	#we update the path of start to end found by A-star search with every step incremented by 1
	for i in range(len(path)):

def search(maze,cost,start,end):
	'''
	   Returns a list of tuples as a path from the given start to the given end in the given maze
	   :param maze:
	   :param cost:
	   :param start:
	   :param end:
	   :return:
	'''

	#create start and end node with initialised values for g,h, and f
	start_node = Node(None, tuple(start))
	start_node.g = start_node.h=start_node.f=0
	end_node = Node(None, tuple(end))
	end_node.g=end_node.h=end_node.f=0

	#Initialize both yet to visit and visited list
	#in the list we will put all the node that are yet to visit for exploration
	#from where we will find the lowest cost node to expand next
	yet_to_visit_list[] = []
	#in this list we will put the nodes that are already explored sot that we don't explore it again
	visited_list = []
	#add the start node
	yet_to_visit_list.append(start_node)

	#adding a stop condition. This is to avoid any infinite loop and stop
	#execution after some reasonable number of steps
	outer_iterations = 0
	max_iterations = (len(maze) // 2) ** 10

	# what squares do we search. search movement is front-left-right-top-bottom-back
	#(6 movements) from every position

	move = [[0,0,-1],  #go up
	        [-1,0,0],  #go left
	        [1,0,0],   #go right
	        [0,-1,0],  #go front
	        [0,1,0],   #go back
	        [0,0,1]]   #go bottom

	#find maze has got how many rows and columns
	no_rows, no_columns = np.shape(maze)

	#Loop until you find the end

	while len(yet_to_visit _the_list)>0:

		#every time any node is referred from yet_to visit the list, counter of limit operation incremented
		outer_itereations += 1

		#Get the current node
		current_node = yet_to_visit_list[0]
		current_index = 0
		for index, item in enumerate(yet_to_visit_list):
			if item.f < current_node.f:
				current_node = item
				current_index = index

		#if we hit this point return the path such as it may be no solution or 
		#computation is too high
		if outer_itereations > max_iterations:
			print("giving up on pathfinding too many iterations")
			return return_path(current_node,maze)

		#Pop the current node out off yet_to_visit_list, add to the visited list
		yet_to_visit_list.pop(current_index)
		visited_list.append(current_node)

		#test if goal is reached or not5 if yes then return the path
		if current_node == end_node:
			return return_path(current_node,maze)

#generate children from all adjacent squares
children=[]

for new_position in move:

	#Get node position
	node_position = (current_node.position[0] + new_position[0],
		             current_node.position[1] + new_position[1],
		             current_node.position[2] + new_position[2])

	#Make sure within range (check if withhin maze boundary)
	if (node_position[0] > (no_rows - 1) or
		node_position[0] < 0 or
		node_position[1] > (no_columns - 1) or
		node_position[1] < 0 or
		node_position[2] > (no_layers - 1) or
		node_position[2] < 0):
		continue

	if maze[node_position[0]][node_position[1]][node_position[2]] != 0:
		continue

	#Create new node
	new_node = Node(current_node,node_position)

	#Append
	children.append(new_node)

for child in children:

	#Child is on the visited list(search entire visited list)
	if len([visited_child for visited_child in visited_list if visited_child == child]) >0:
		continue

	# Create the f,g, and h values
	child.g = current_node.g + cost
	##Heuaristic costs calculated here, this is using euclidean distance
	child.h = (((child.position[0] - end_node.position[0]) ** 2) + 
		        ((child.position[1] - end_node.position[1]) ** 2) +
                ((child.position[2] - end_node.position[2]) ** 2))

	child.f = child.g + child.h

	# if the child is already in the yet to visit list and g cost is already lower
	if len([i for i in yet_to_visit-list if child == i and child.g >i.g]) > 0:
		continue

	#Add the child to the yet to visit list
	yet_to visit_list.append(child)

if __name__ = '__main__':

	path = search(maze,cost,start,end)
	print(path)


