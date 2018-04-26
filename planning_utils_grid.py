import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi
from bresenham import bresenham
from queue import PriorityQueue
import sys
import pkg_resources
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
    centers = []
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
            centers.append([north - north_min, east - east_min])

    return grid, centers, int(north_min), int(east_min)

def generate_graph(grid, points):
    print(len(points))
    vornoi_graph = Voronoi(points)
    graph = nx.Graph()
    edges = []
    for v in vornoi_graph.ridge_vertices:
        p1 = vornoi_graph.vertices[v[0]]
        p2 = vornoi_graph.vertices[v[1]]
        
        # optimizing the path using bresenham method
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the graph
        if not hit:
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            # dist = np.linalg.norm(np.array(p2) - np.array(p1))
            dist = 1
            graph.add_edge(p1, p2, weight=dist)
            edges.append([p1,p2])
    return graph, edges
    

def a_star(graph, h, start, goal):

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
            for next_node in graph[current_node]:
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

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def closest_point(p,graph):
    min_dist = 1000000
    point = None
    for node in graph.nodes:
        dist = np.linalg.norm(np.array(node) - np.array(p))
        if dist < min_dist:
            point = node
            min_dist = dist
    return point

def collinearity_check(p1, p2, p3): 
    collinear = False
    epsilon = 0.0001
    det = p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]- p2[1])
    if det <= epsilon:
        collinear = True
    return collinear

def prune(path):
	i=0
	while i+2 < len(path):
		point_1 = path[0]
		point_2 = path[i+1]
		point_3 = path[i+2]
		if collinearity_check(point_1, point_2, point_3):
			path.remove(path[i+1])
		i+=1

	for i in range(len(path)):
		path[i] = (int(path[i][0]), int(path[i][1]))

	return path

