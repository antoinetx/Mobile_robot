#######################################
# IMPORT AND LIB
#######################################

import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from matplotlib import colors

from robot import Robot
from Map import Map


#######################################
# UTILS FUNCTIONS 
#######################################

# Display functions

def create_empty_plot(lenght):
    """
    Helper function to create a figure of the desired dimensions & grid
    
    :param max_val: dimension of the map along the x and y dimensions
    :return: the fig and ax objects.
    """
    fig, ax = plt.subplots(figsize=(7,7))
    
    major_ticks_x = np.arange(0, lenght[0]+1, 5)
    minor_ticks_x = np.arange(0, lenght[0]+1, 1)
    ax.set_xticks(major_ticks_x)
    ax.set_xticks(minor_ticks_x, minor=True)
    major_ticks = np.arange(0, lenght[1]+1, 5)
    minor_ticks = np.arange(0, lenght[1]+1, 1)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)
    #ax.grid(which='minor', alpha=0.2)
    #ax.grid(which='major', alpha=0.5)
    ax.set_ylim([-1,lenght[1]])
    ax.set_xlim([-1,lenght[0]])
    ax.grid(True)
    
    return fig, ax

def display_map(lenght , occupancy_grid , visitedNodes ,  path , start, goal):
    cmap = colors.ListedColormap(['white', 'red']) # Select the colors with which to display obstacles and free cells
    # Displaying the map
    print("LENNNN", lenght)
    fig_astar, ax_astar = create_empty_plot(lenght)
    ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)


    # Plot the best path found and the list of visited nodes
    ax_astar.scatter(visitedNodes[0], visitedNodes[1], marker="o", color = 'orange');
    ax_astar.plot(path[0], path[1], marker="o", color = 'blue');                        #OPTIMAL PATH  0(x) 1(y)
    ax_astar.scatter(start[0], start[1], marker="o", color = 'green', s=200);
    ax_astar.scatter(goal[0], goal[1], marker="o", color = 'purple', s=200);
  
# A ALGORITHM 

def _get_movements_4n():
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]

# RECONSTRUCT PATH

def reconstruct_path(cameFrom, current):
    """
    Recurrently reconstructs the path from start node to the current node
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n 
                     currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path

def A_Star(start, goal, h, coords, occupancy_grid, lenght, movement_type="4N"):
    """
    A* for 2D occupancy grid. Finds a path from start to goal.
    h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param start: start node (x, y)
    :param goal_m: goal node (x, y)
    :param occupancy_grid: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    
    # -----------------------------------------
    # DO NOT EDIT THIS PORTION OF CODE
    # -----------------------------------------
    
    # Check if the start and goal are within the boundaries of the map
    
    print("start_x",start[0])
    print("start_x",start[1])
    print("goal_y",goal[0])
    print("goal_y",goal[1])
    print("lenght_x",lenght[0])
    print("lenght_y",lenght[1])
    
    print("test ifeee")
    
    if ((0 < start[0] <= lenght[0]) &  (0 < start[1] <= lenght[1])):
        print("C'est bon")
        
    
    assert  0 <= start[0] <= lenght[0] and  0 <= start[1] <= lenght[1], "start not contained in the map"
    assert 0 <= goal[0] <= goal[0] and  0 <= goal[1] <= goal[1], "goal not contained in the map"
    
    #for point in [start, goal]:
    #    for coord in point:
    #        assert coord>=0 and coord< max(lenght), "start or end goal not contained in the map"
    
    # check if start and goal nodes correspond to free spaces
    if occupancy_grid[start[0], start[1]]:
        raise Exception('Start node is not traversable')

    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    
    # get the possible movements corresponding to the selected connectivity
    if movement_type == '4N':
        movements = _get_movements_4n()
    elif movement_type == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')
    
    # --------------------------------------------------------------------------------------------
    # A* Algorithm implementation - feel free to change the structure / use another pseudo-code
    # --------------------------------------------------------------------------------------------
    
    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]
    
    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]

    # while there are still elements to investigate
    while openSet != []:
        
        #the node in openSet having the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet
        
        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)
        
        #for each neighbor of current:
        for dx, dy, deltacost in movements:
            
            neighbor = (current[0]+dx, current[1]+dy)
            
            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue
            
            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                continue
                
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            
            if neighbor not in openSet:
                openSet.append(neighbor)
                
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet


#######################################
# PATH COMPUTATION AND UPDATE 
#######################################
def path_computation(start , goal , lenght, occupancy_grid):
    x,y = np.mgrid[0:lenght[0]:1, 0:lenght[1]:1]
    print("youhouuuuu")
    print(x.shape)
    print(y.shape)
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x; pos[:, :, 1] = y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in pos])

    # Define the heuristic, here = distance to goal ignoring obstacles
    h = np.linalg.norm(pos - goal, axis=-1)
    h = dict(zip(coords, h))

    # Run the A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid, lenght, movement_type="8N")

    path = np.array(path).reshape(-1, 2).transpose()
    print(len(visitedNodes))
    visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()
    print(visitedNodes)
    return path, visitedNodes


def path_update(pos, err_pos, path, current):
    # Next Goal Update
    if ((pos - path(current)) < (err_pos,err_pos)):
        return current+1
    else:
        return current  


#######################################
# INITIALISATIONS JUSTE POUR TESTS
#######################################

"""
robot_start = (0,0) 
robot_goal = (43,33) 

Paris = Map(5, 50)
george = Robot(robot_start,robot_goal)
"""

# Decoment for a map different than 0 -----------------
"""     
fig, ax = create_empty_plot(Paris.get_lenght())
max_val = 50 # Size of the map

# Creating the occupancy grid
np.random.seed(0) # To guarantee the same outcome on all computers
data = np.random.rand(Paris.get_lenght(), Paris.get_lenght()) * 20 # Create a grid of 50 x 50 random values
cmap = colors.ListedColormap(['white', 'red']) # Select the colors with which to display obstacles and free cells
# Converting the random values into occupied and free cells
limit = 12 
occupancy_grid = data.copy()
occupancy_grid[data>limit] = 1
occupancy_grid[data<=limit] = 0

Paris.update_map(occupancy_grid)
"""
#---------------------------

"""
#######################################
# MAIN FOR EST 
#######################################


path, visitedNodes = path_computation(george.get_start() , george.get_goal() , Paris.get_lenght(), Paris.get_map())

george.set_path(path) 
george.set_visit_nodes(visitedNodes) 

display_map(Paris.get_lenght(),  Paris.get_map(),  george.get_visit_nodes(), george.get_path(), george.get_start(), george.get_goal())


plt.show()

"""


