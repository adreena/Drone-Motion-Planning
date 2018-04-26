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

Partitioning the grid into regions based on distance to obstacle centers helps identifying the edges in the graph to represent feasible paths for navigating around obstacles which is performed using [Voronoi Diagram](https://en.wikipedia.org/wiki/Voronoi_diagram). [code: planning_utils.py -> generate_graph]  

#### 2. Start , Destination and Path


And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


