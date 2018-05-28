## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes a framework and support functions to move the quad from a start location to a goal location through a series of intermediate waypoints. Without modification, the default [motion_planning.py](motion_planning.py) code will bring the quad from map center (316, 445) to a nearby goal (326, 455). This is done by an additional `PLANNING` phase between the `ARMING` and `TAKEOFF` phases (comparing to [backyard_flyer_solution.py](backyard_flyer_solution.py)). During the `PLANNING` phase the method `plan_path()` is called, which further load the colliders data and run the A* planning using the code provided in [planning_utils.py](planning_utils.py).

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
`lat0` and `lon0` can be extracted from the first line of [colliders.csv](colliders.csv). As shown in **line 124-127** of [motion_planning.py](motion_planning.py), I extract `lat0` and `lon0` as floating point values and use the `self.set_home_position()` method to set global home.
```python
lat0, lon0 = open('colliders.csv').readline().replace("lat0", "").replace("lon0", "").strip().split(",")
lat0 = float(lat0)
lon0 = float(lon0)
self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
This is done by the following code in **line 129** of [motion_planning.py](motion_planning.py):
```python
current_local_position = global_to_local(self.global_position, self.global_home)
```

#### 3. Set grid start position from local position
In [motion_planning.py](motion_planning.py) we define grid start position in **line 142**:
```python
grid_start = (int(current_local_position[0]-north_offset), int(current_local_position[1]-east_offset))
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. In **line 130 & 145** I convert the latitude and longtitude stored in `self.global_goal` (which was passed in via the `__init__` function) to grid position:
```python
goal_local_position = global_to_local(self.global_goal, self.global_home)
grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I modify the code in [planning_utils.py](planning_utils.py) to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2). More specifically, in the `Action` enum class, I add four diagonal directions. And the `valid_actions()` method is also modified to incorporate the new directions as following:
```python
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    for action in list(Action):
        nx = x + action.delta[0]
        ny = y + action.delta[1]
        if 0 <= nx < n and 0 <= ny < m and grid[nx, ny] != 1:
            valid_actions.append(action)

    return valid_actions
```

#### 6. Cull waypoints 
To prune path of unnecessary waypoints, in **line 150-170** I use collinearity test to eliminate intermediate waypoints, just as we've done in the excercise of previous lessons.



### Execute the flight
#### 1. Does it work?
It works! Please find the video under the `misc` folder: [video.mp4](misc/video.mp4).

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
