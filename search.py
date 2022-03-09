# Basic searching algorithms

from cmath import inf

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h=0, parent=None):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = parent    # previous node

    ################### Newly Defined functions ####################
    def get_path(self):
        '''
        Returns path from start_node (node with parent = None) to goal_node

        return:
        path - Path (list)
        '''
        node = self
        path = []
        while node.parent != None:
            path.append([node.row, node.col])
            node = node.parent
        path.append([node.row, node.col])
        path.reverse()
        return path

    def explore_neighbors(self, grid):
        '''
        Returns neighbours (4 connectivity) for the given node.
        Order followed: right, down, left, up

        arguments:
        grid - information about the grid is needed to know if a node is a valid neighbour or an obstacle or a boundary condition

        return:
        neighbours - list of coordinates of valid neighbours in the form [row, col]
        '''
        dc = [1, 0, -1, 0]
        dr = [0, 1, 0, -1]

        neighbours = []
        for i in range(4):
            row = self.row + dr[i]
            col = self.col + dc[i]
            if ((row >= 0) and (col >= 0) \
                and (row < len(grid)) and (col < len(grid[0])) \
                and grid[row][col]==0):
                neighbours.append([row, col])
        return neighbours
    
    #################################################################


############ Common Function to all 4 algorithms ###############

def initialize(start, goal, grid):
    ''' This function initializes the start node, goal node, a visited matrix with False values and 
        it also checks if the start or goal node are obstacles to handle errors in input.
    '''
    obs = 0
    start_node = Node(start[0], start[1], grid[start[0]][start[1]])
    goal_node = Node(goal[0], goal[1], grid[goal[0]][goal[1]])

    if(start_node.is_obs or goal_node.is_obs):
        obs = 1
    
    visited_matrix = [[False for i in range(len(grid[0]))] for j in range(len(grid))]

    return start_node, goal_node, visited_matrix, obs

#################################################################

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    st_node, goal_node, visited, obs = initialize(start, goal, grid)
    if obs:
        print("No path found")
        return path, steps

    visited[st_node.row][st_node.col] = True

    Q = []
    Q.append(st_node)

    while len(Q) > 0:
        u = Q.pop(0)                    # First In First Out operation using array
        steps += 1
        if ([u.row, u.col] == [goal_node.row, goal_node.col]):
            found = True
            goal_node.parent = u.parent
            break
        for n in u.explore_neighbors(grid):
            if (visited[n[0]][n[1]] == True):       # Visited neighbors are skipped
                continue
            v = Node(n[0], n[1], grid[n[0]][n[1]], parent=u)
            Q.append(v)
            visited[v.row][v.col] = True

    path = goal_node.get_path()
    
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    st_node, goal_node, visited, obs = initialize(start, goal, grid)
    if obs:
        print("No path found")
        return path, steps

    found, steps = dfs_recurse(grid, st_node, goal_node, visited, steps, found)

    path = goal_node.get_path()

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps

def dfs_recurse(grid, u, goal_node, visited, steps, found):
    visited[u.row][u.col] = True
    steps += 1
    if ([u.row, u.col] == [goal_node.row, goal_node.col]):
        found = True
        goal_node.parent = u.parent
        return found, steps
    for n in u.explore_neighbors(grid):
        if (visited[n[0]][n[1]] == True):               # If already visited, skip the neighbor
            continue
        v = Node(n[0], n[1], 0, parent=u)
        found, steps = dfs_recurse(grid, v, goal_node, visited, steps, found)
        if (found == 1):
            return found, steps
    return found, steps

##################################################################################
########### Newly Defined Functions used in Djikstra and A* algorithms ###########

def compute_heuristics(node, goal_node):
    '''
        Returns h which is the Manhattan Distance between the current node and the goal node
    '''
    h = abs(goal_node.row - node.row) + abs(goal_node.col - node.col)
    return h

def traverse_graph(st_node, goal_node, grid, visited, steps, heuristics=False):
    '''
        This function is used for implementing Dijkstra and A* algorithms. Since the implementation is similar the only parameter that changes
        is the cost f(x), which this function updates based on the input to the parameter 'heuristics'. If we are implementing A* algorithm, we set heuristics to True
        which means the algorithm will include the heursitics calculations in order to compute the cost f(x)
    '''
    st_node.g = 0
    if heuristics == True:              # For A* algorithm, heuristics needs to be computed
        st_node.h = compute_heuristics(st_node, goal_node)
    st_node.cost = st_node.g + st_node.h
    goal_node.g = inf
    goal_node.cost = inf
    
    Q = []
    Q.append(st_node)

    while len(Q) > 0:
        Q.sort(key=lambda x: x.cost)
        u = Q.pop(0)
        if (visited[u.row][u.col] == True):         # If object is already visited (Closed list), skip this iteration
            continue
        visited[u.row][u.col] = True                # Mark for closed list
        steps += 1
        if ([u.row, u.col] == [goal_node.row, goal_node.col]):
            goal_node.g = u.g
            goal_node.cost = u.cost
            goal_node.parent = u.parent
            found = True
            break
        for n in u.explore_neighbors(grid):         # Explore neighbors
            if (visited[n[0]][n[1]] == True):       # If neighbors are visited (in closed list), skip
                continue
            v = Node(n[0], n[1], grid[n[0]][n[1]], parent=u)
            v.g = u.g + 1                           # Can change increment to include weights
            if heuristics == True:                  # For A* algorithm, heuristics needs to be computed
                v.h = compute_heuristics(v, goal_node)
            v.cost = v.g + v.h
            Q.append(v)                             # Add to queue with new cost, lowest will be popped first

    return goal_node, steps, found

##################################################################################

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    st_node, goal_node, visited, obs = initialize(start, goal, grid)
    if obs:
        print("No path found")
        return path, steps

    goal_node, steps, found = traverse_graph(st_node, goal_node, grid, visited, steps, heuristics=False)

    path = goal_node.get_path()

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps

def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    st_node, goal_node, visited, obs = initialize(start, goal, grid)
    if obs:
        print("No path found")
        return path, steps
    
    goal_node, steps, found = traverse_graph(st_node, goal_node, grid, visited, steps, heuristics=True)

    path = goal_node.get_path()

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
