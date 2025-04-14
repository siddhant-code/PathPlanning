# ENPM 661 - Planning for Autonomous Robots
## Project-3: Phase-1

### Students:

1) Siddhant Pramod Deshmukh (121322463)
2) Pon Aswin Sankaralingam (121322517)
3) Venkata Madhav Tadavarthi (121058768)

### Repository:
https://github.com/siddhant-code/PathPlanning.git

## Introduction:

Implementing A* Algorithm to find a path between the start and end point on a given map for Turtle bot 3 in Falcon simulator

## Dependencies:

You require the following libraries to work with the python code:

- numpy
- matplotlib
- sympy
- math
- queue
- moviepy
- time
- opencv

You can install these libraries by:

```
pip install <library_name>
```

## Usage

Clone or download the repository

```
git clone https://github.com/siddhant-code/PathPlanning.git
```

## Installation and setup 

Make sure Falconsim and ROS2 is installed in your system

Update Launch File Paths
location: ROS2/falcon_turtlebot3_project_ws/src/astar_falcon_planner/launch/ros_falcon_astar.launch.py

```
ros_falcon_astar.launch.py
```

Edit ros_falcon_astar.launch.py:

Update cwd= to your FalconSim install path (e.g., /home/username/duality/falconsim/)
Update scenario= to the full path of AMRPathPlanning.usda


Go to the Part-2 folder inside a* folder:

```
cd path_planning_algorithms/astar/Phase2/Part-2/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Launch Simulation
Use the command below to launch your simulation.

All values must be filled in by you based on your planning map and robot configuration.

ros2 launch astar_falcon_planner ros_falcon_astar.launch.py \
    start_position:=[0.0, 0.0, 0.0] \
    end_position:=[5.2, 0.0, 0.0] \
    robot_radius:=0.0 \
    clearance:=0.0 \
    delta_time:=0.0 \
    wheel_radius:=0.0 \
    wheel_distance:=0.0 \
    rpms:=[0.0, 0.0]

## Instructions

When you run the program, it will first display the workspace map (250x600) showing the obstacles, wall and the clearance region.

![map](./assets/map.png)

After viewing the map, press Q key to close the window and continue.

### User Input:

- Start Position: ($x_s,y_s,\theta_s$) 
    - "Enter the start coordinates in form x,y,theta: "

- Goal Position: ($x_g,y_g,\theta_g$) 
    - "Enter the goal coordinates in form x,y,theta: "

- Step-size: ($1 < step < 10$)
    - "Enter the step size: "

The script will validate whether the input and goal coordinates are within free space. Otherwise prints a message "-- Point inside obstacle space, please chose different starting point --" and "-- End inside obstacle space, please chose different starting point --" until valid coordinates are provided by the user. 

If the step size is not between 1 to 10, it prints a message "Step size value should be a value between 1 and 10" and asks the user to provide step size untill valid value is given.

### Example:

![output](./assets/output.png)

![output](./assets/output.gif)


