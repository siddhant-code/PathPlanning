# ENPM 661: Path Planning for Autonomous Robots
### Instructions for Project3- Phase2

## Part-1

## Map Dimensions

All dimensions are in milimeters.

![map](map.png)


## Clone the reposiory

```sh
git clone https://github.com/siddhant-code/PathPlanning.git
```

## Install ROS2 Humble (If not already installed)
Follow [this guide](https://docs.ros.org/en/humble/Installation.html) Then install colcon:

```sh
sudo apt install python3-colcon-common-extensions
```

## Build the workspace

```sh
cd path_planning_algorithms/astar/Phase2/project3_ws/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```


## Test Setup

Launch Environment

```sh
ros2 launch turtlebot3_project3 competition_world.launch
```

You should see the turtlebot3 along with the maze in gazebo

![gazebo](gazebo.png)

Open another terminal:

```sh
ros2 run turtlebot3_project3 controller.py
```


# Error

* Pynput Module Not Found

```sh
pip install pynput
```