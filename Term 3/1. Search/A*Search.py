#Python


# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0]]

# Heuristic function is the number of actions required from each
# node to reach the goal.

heuristic = [[9, 8, 7, 6, 5, 4],
	         [8, 7, 6, 5, 4, 3],
	         [7, 6, 5, 4, 3, 2],
	         [6, 5, 4, 3, 2, 1],
	         [5, 4, 3, 2, 1, 0]]


init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']


def search(grid,init,goal,cost):

	# Initialize a grid indicating already checked locations
	
	closed = [[ 0 for row in range(len(grid[0]))] for col in range(len(grid))]

	# expand is the grid where each node has the value of what order it was expanded
	expand = [[ -1 for row in range(len(grid[0]))] for col in range(len(grid))]
	
	closed[init[0]][init[1]] = 1

	numbOfExpansions = 0

	# Stores the action taken to reach the current node
	action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

	x = init[0]
	y = init[1]
	g = 0
	f = g + heuristic[x][y]

	open = [[f,g,x,y]]
	
	found = False
	stopSearch = False

	while not found and not stopSearch:

		if len(open) == 0:
			stopSearch = True
			print('Can\'t find Goal')

		else:
			# Expand
			
			# Remove and expand onthe ndoe from the list that has the smallest g value
			# We can do this by sorting the array based on the g value, and then removing the one with the smallest g and expand
			
			open.sort()
			open.reverse()
			next = open.pop()
			
			f = next[0]
			g = next[1]
			x = next[2]
			y = next[3]

			expand[x][y] = numbOfExpansions			
			numbOfExpansions += 1
			

			# is this the goal position
			
			if x == goal[0] and y == goal[1]:
				found = True
				print('Found the goal')
				print(next)
			else:
				for i in range(len(delta)):
					x2 = x + delta[i][0]
					y2 = y + delta[i][1]
					if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
						if closed[x2][y2] == 0 and grid[x2][y2] == 0:
							g2 = g + cost
							f2 = g2 + heuristic[x2][y2]
							open.append([f2, g2, x2, y2])
							# newly searched point
							closed[x2][y2] = 1
							action[x2][y2] = i


	policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]

	x = goal[0]
	y = goal[1]

	policy[x][y] = '*'

	while x != init[0] or y != init[1]:
		x2 = x - delta[action[x][y]][0]
		y2 = y - delta[action[x][y]][1]

		policy[x2][y2] = delta_name[action[x][y]]
		x = x2
		y = y2


	return policy


policy = search(grid,init,goal,cost)

for i in range(len(policy)):
	print(policy[i])
