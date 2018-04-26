## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---
The goals / steps of this project are the following:

 - Step 1: collecting obstacles data
 - Step 2: creating a grid based on obstacles information and localizing the coordinates
 - Step 3: partitioning the grid into regions based on distance to obstacle centers, the edges in the graph represent feasible paths to navigate around those point obstacles 
 - Step 4: Setting feasible start/goal points within the grid region
 - Step 5: performing a search algorithm to find a directed path between start and goal using heuristics
 - Step 6: pruning the path to improve drone motions 
 
---
### Model Architecture

The main scripts are provided in `motion_planning.py` and `planning_utils.py`. 

#### 1. Model states

Model is listening to multiple events including changes in local position, local velocity and states. The simulated drone follows a sequence of state changes to be functioning properly by first going to MANUAL, then ARMING so the rotors start wrking and at that state it is ready to plan on how to get a destination form its current location as starting point and take off. After reaching the destination it would disarm and land in the goal point.

#### 2. Obstacles

To find a safe path from start to destination, drone needs to be aware of obstacles in its surrounding and this information is provided in `colliders.csv` file which contains the 2.5D map of the simulator environment. The initial step is to create a grid of obstalces and localize coordinates to the grid. [code: planning_utils.py -> create_grid]

Partitioning the grid into regions based on distance to obstacle centers helps identifying the edges in the graph to represent feasible paths for navigating around obstacles which is implemented using [Voronoi Diagram](https://en.wikipedia.org/wiki/Voronoi_diagram). Voronoi provide ridge_vertices which defines the midline in free space between the obstacles and is useful for creating an efficient graph of edges connecting the ridge_vertices. [Bresenham](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) method is an efficient way to determine the points connecting the ridge_vertices with a close approximation to a straight line. [code: planning_utils.py -> generate_graph]

[Grid Image Placeholder]


#### 3. Set Start/Goal

The model is able to recieve global coordinates (latitude, longitude) information about the start and goal position and convert them to local coordinates by considering the obstacle grid offsets. [code: planning_utils.py -> localize_point]

Then the closes points to these points on the graph are identified based on the minimum distance to the nodes. [code: planning_utils.py -> closest_point]


#### 4. Path Searching

Search algorithm used in this model is [A*](https://en.wikipedia.org/wiki/A*_search_algorithm), it searches among all possible paths to the goal for the minimum cost. It constructs a tree of paths from the starting node, expanding paths one step at a time, until one of its paths ends at the goal node by determining which of its partial paths to expand into one or more longer paths. Each partial path is assigned an estimate cost that's the sum of node's current_cost and the heuristic current_cost + path_cost + heuristic. Heuristic used in this model is the distance between goal and the node. [code: planning_utils.py -> a_star]

#### 5. Path Pruning

Once model found the path and returned the waypoints, model does further optimization using colliearnity which is to eliminate points that lie on a single straight line between 2 other points. Pruning reduces the number of uncessary stops/transitions for the drone.[code: planning_utils.py -> collinearity_check] 

Waypoints are then casted to integer and given a heading direction [code: planning_utils.py -> collinearity_check] 


And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)


### Other experiments / Future works

I experimented random sampling to generate 300 random points within the grid and removed the ones which were not in the safety distance from the obstacles. I then used KDTree to create a graph connecting the good nodes, there is a noticable processing time for this approach and I prefered using the grid solution. [code: random_path_planning.ipynb] 
![Random Sampling](./misc/high_up.png)




