# ENPM 661 - Planning for Autonomous Robots
## Project-3: Phase-1

### Students:

1) Siddhant Pramod Deshmukh (121322463)
2) Pon Aswin Sankaralingam (121322517)
3) Venkata Madhav Tadavarthi (121058768)

### Repository:
https://github.com/siddhant-code/PathPlanning.git

## Introduction:

Implementing A* Algorithm to find a path between the start and end point on a given map for a mobile
robot.

## Dependencies:

You require the following libraries to work with the python code:

- numpy
- matplotlib
- sympy
- itertools
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

Go to the A* folder:

```
cd path_planning_algorithms/astar/
```

You can run the python file by the following command:

```
python3 a_star_Siddhant_PonAswin_VenkataMadhav.py
```

## Instructions

When you run the program, it will first display the workspace map (250x600) showing the obstacles, wall and the clearance region.

![map](./assets/map.png)

After viewing the map, press Q key to close the window and continue.

### User Input:

- Start Position: ($x_s,y_s,\theta_s$) 
    - "Enter the start coordinates in form x,y,theta: "

- Goal Position: ($x_g,y_g,\theta_g$) 
    - "Enter the goal coordinates in form x,y,theta: "

- Step-size: ($1 < 10 < 10$)
    - "Enter the step size: "

The script will validate whether the input and goal coordinates are within free space. Otherwise prints a message "-- Point inside obstacle space, please chose different starting point --" and "-- End inside obstacle space, please chose different starting point --" until valid coordinates are provided by the user. 

If the step size is not between 1 to 10, it prints a message "Step size value should be a value between 1 and 10" and asks the user to provide step size untill valid value is given.

### Example:

![output](./assets/output.png)

![output](./assets/output.gif)


