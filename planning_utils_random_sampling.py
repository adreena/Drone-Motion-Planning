from enum import Enum
from queue import PriorityQueue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import networkx as nx

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions

def generate_samples(north, east, data, obstacle_tree,polygons, target_altitude, start, goal):
    n_sample= 300
    x_samples = np.random.uniform(north['min'], north['max'], n_sample)
    y_samples = np.random.uniform(east['min'], east['max'], n_sample)
    z_samples = np.random.uniform(0, target_altitude, n_sample)

    random_samples = list(zip(x_samples,y_samples,z_samples))
    random_samples.append(start)
    random_samples.append(goal)

    nodes = []
    max_xy = 2*np.max((data[:,3], data[:,4]))
    for sample in random_samples:
        collision = False
        idx = list(obstacle_tree.query_radius(np.array([sample[0],sample[1]]).reshape(1,-1), r=max_xy)[0])
        for obstacle_id in idx:
            poly, height = polygons[obstacle_id]
            # polygon heigh is already gapped with safety_distance
            if poly.contains(Point(sample)) and sample[2] <= height :
                collision = True
                if sample == goal:
                    print('GOAL REMOVED')
        if collision is False:
            nodes.append(sample)
    return nodes, random_samples

def create_obstacle_graph(north_info, east_info, data, safety_distance):
    polygons = []
    polygons_center = []
    for row in data:
        north, east, alt, d_north, d_east, d_alt = row
        obstacle = [
            int(np.clip(north - d_north - safety_distance - north_info['min'], 0, north_info['size']-1)),
            int(np.clip(north + d_north + safety_distance - north_info['min'], 0, north_info['size']-1)),
            int(np.clip(east - d_east - safety_distance - east_info['min'], 0, east_info['size']-1)),
            int(np.clip(east + d_east + safety_distance - east_info['min'], 0, east_info['size']-1)),
        ]
        corner_0= (obstacle[0], obstacle[2])
        corner_1= (obstacle[0], obstacle[3])
        corner_2= (obstacle[1], obstacle[3])
        corner_3= (obstacle[1], obstacle[2])
        poly = Polygon([corner_0, corner_1, corner_2, corner_3])
        height = alt + d_alt + safety_distance
        polygons.append((poly,height))
        polygons_center.append((poly.centroid.x, poly.centroid.y))

    obstacle_tree = KDTree(polygons_center, metric='euclidean')
    return obstacle_tree, polygons

def get_data_coords(data):
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    
    return north_min, north_max, east_min, east_max, north_size, east_size

def can_connect(node1,node2, polygons):
    line = LineString([node1, node2])
    for poly, height in polygons:
        # polygon heigh is already gapped with safety_distance
        if poly.crosses(line) and min(node1[2],node2[2]) <= height: 
            return False
    return True

def create_graph(nodes,polygons):
    sample_tree = KDTree(nodes)
    graph = nx.Graph()
    for node in nodes:
        neighbors = list(sample_tree.query([node], k=5, return_distance=False)[0])
        for index  in neighbors:
            other_node = nodes[index]
            if node!=other_node and can_connect(node,other_node, polygons):
                 graph.add_edge(node, other_node, weight=1)
    
    return graph, sample_tree

def a_star_(graph, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            other_nodes = graph[current_node]
            for next_node in other_nodes:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def find_closest_node(node, graph, sample_tree, nodes, polygons):
    neighbors = list(sample_tree.query([node], k=10, return_distance=False)[0])
    closest_node = None
    if len(neighbors)>0:
        min_dist= 10000
        for ind in neighbors:
            neighbor = nodes[ind]
            all_nodes = list(graph.nodes)
            if neighbor in list(graph.nodes):
                dist = np.linalg.norm(np.array(neighbor) - np.array(node))
                if dist < min_dist and can_connect(node,neighbor, polygons):
                    closest_node = neighbor
                    min_dist = dist
        return closest_node
    else: 
        return None

def get_random_point():
    point = list(graph.nodes)[np.random.randint(len(graph.nodes))]
    global_home = np.array([-122.39745 ,  37.79248 ,   0.     ])


    temp_point_1=(point[0] + north_offset, point[1] + east_offset, 0)
    # temp_point_1=(point[0] , point[1] , 0)
    geodetic_current_coordinates = local_to_global(temp_point_1, global_home)
    local_position = global_to_local(geodetic_current_coordinates, global_home)
    global_position  = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))

    print("Point from the graph: ",point)
    print("Point localized: ",local_position)
    print("Point global: ", global_position)
    return point

def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

