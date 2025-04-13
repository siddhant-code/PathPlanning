import time
import math
from functools import lru_cache
import heapq
import numpy as np
from sympy import symbols
from moviepy.editor import ImageSequenceClip
import cv2
import matplotlib.pyplot as plt

# Defining symbols
x,y,z,a,b,r = symbols("x,y,z,a,b,r")

# class to define lines
class Line():
  def __init__(self,equation,symbol) -> None:
    self.equation = equation
    self.symbol = symbol

  # Function to check point lies on correct side of line
  def check(self,point):
    point = {x:point[0],y:point[1]}
    value = self.equation.xreplace(point)
    if self.symbol == "g":
      return value >= 0
    else:
      return value < 0

# class to define shape
class Shape():
  def __init__(self,lines) -> None:
    self.lines = lines

  #Function to add line to define shape
  def add_line(self,line:Line):
    self.lines.append(line)

  # Function to check whether given point lies inside the shape
  def check_point_inside_shape(self,point):
    verdict = True
    for line in self.lines:
      verdict = verdict and line.check(point)
      if not verdict:
        break
    return verdict

# class to define shape collection by combining shapes
class ShapeCollection():
  def __init__(self,shapes) -> None:
    self.shapes = shapes

  # Function to add shape
  def add_shape(self,shape:Shape):
    self.shapes.append(shape)

  # Function to check if point lies inside shape collection
  def check_point_inside_shape_collection(self,point):
    verdict = False
    for shape in self.shapes:
      verdict = verdict or shape.check_point_inside_shape(point)
      if verdict:
        break
    return verdict

height = 300
width = 740
WHEEL_DIAMETER = 6.6 # in cm
ROBOT_RADIUS = 22.0 # in cm
WHEEL_DISTANCE = 28.7 # in cm
WHEEL_RADIUS = WHEEL_DIAMETER/2
LOW_RPM = 50 # default radian/s
HIGH_RPM = 100 # default radian/s
DISTANCE_THRESHOLD = 2 # cm
DELTA_TIME = 1
BACKGROUND_COLOR = (232,215,241)
OBSTACLE_COLOR = (0,0,0)
CLEARANCE_COLOR = (100,100,100)
ASTAR_MAP = None
CLEARANCE = 10
OFFSET_X = 0
OFFSET_Y = height/2

action_list = {
    'low_left': (0,LOW_RPM),
    'high_left': (0,HIGH_RPM),
    'low_right': (LOW_RPM,0),
    'high_right': (HIGH_RPM,0),
    'low_straight': (LOW_RPM,LOW_RPM),
    'mid_left': (LOW_RPM,HIGH_RPM),
    'mid_right': (HIGH_RPM,LOW_RPM),
    'high_straight': (HIGH_RPM,HIGH_RPM)
}

# Function to flip the y (origin)
def flip_y(y):
    return height - y - 1

@lru_cache
def calculate_wheel_velocity(rpm, radius = WHEEL_RADIUS):
  return rpm * 2 * math.pi * radius / 60

@lru_cache
def calculate_velocity(left_rpm,right_rpm):
  left_wheel_velocity = calculate_wheel_velocity(left_rpm) # mm / sec
  right_wheel_velocity = calculate_wheel_velocity(right_rpm)
  linear_velocity =  (left_wheel_velocity + right_wheel_velocity) / 2
  angular_velocity = (right_wheel_velocity - left_wheel_velocity) / WHEEL_DISTANCE
  if left_rpm == right_rpm:
    icc_radius = None
  else:
    icc_radius = ( WHEEL_DISTANCE * linear_velocity )/(right_wheel_velocity - left_wheel_velocity)
  return {"linear":linear_velocity,"angular":angular_velocity,"icc_radius":icc_radius}

@lru_cache
def get_next_position(x,y,theta,left_rpm,right_rpm,dt=DELTA_TIME):
    linear_velocity,angular_velocity, R = calculate_velocity(left_rpm,right_rpm).values()
    if angular_velocity == 0:
        x_new,y_new,theta_new = x + linear_velocity*math.cos(theta)*dt, y + linear_velocity*math.sin(theta)*dt, theta
    else:
        angle_turned = angular_velocity*dt
        k = 2 * R * math.sin(angle_turned/2)
        x_new = x + k*math.cos(theta + angle_turned/2)
        y_new = y + k*math.sin(theta + angle_turned/2)
        theta_new = theta + angle_turned
        theta_new = check_angle_limit(theta_new)
        
    return (x_new// 0.4) * 0.4,(y_new//0.4) * 0.4,theta_new

def get_children(x,y,theta):
    return {action:get_next_position(x,y,theta,*action_list[action]) for action in action_list.keys()}

def calculate_heuristic(point1,point2):
    return math.hypot(point1[0] - point2[0],point1[1] - point2[1]) / calculate_wheel_velocity(HIGH_RPM)

def is_obstacle(point,space_mask):
    x , y = int(point[0]) , int(point[1])
    h,w = space_mask.shape
    return  x < 0 or x >= w or y < 0 or y >= h or not space_mask[y][x]


def is_goal_node(node,goal_node,threshold = DISTANCE_THRESHOLD):
    return math.hypot(node[0] - goal_node[0],node[1] - goal_node[1]) < threshold

@lru_cache
def get_vertices_for_curve(point1,action,resolution=10):
    x,y,theta = point1
    points = [(x,y)]
    for _ in range(resolution):
        x,y,theta = get_next_position(x,y,theta,*action_list[action],DELTA_TIME/resolution)
        points.append((x,y))
    return points

def transform_coordinate(position):
    if len(position) == 2:
        theta = 0
    else:
        theta = position[2]
    return (position[0] + OFFSET_X,position[1] + OFFSET_Y,theta)

def inverse_transform_coordinate(position):
    if len(position) == 2:
        theta = 0
    else:
        theta = position[2]
    return (position[0] - OFFSET_X,position[1] - OFFSET_Y,theta)
    

@lru_cache
def check_angle_limit(dtheta):
    if dtheta >= math.pi :
        dtheta = -2*math.pi + dtheta
    elif dtheta < -math.pi:
        dtheta = 2*math.pi + dtheta
    return dtheta

def check_trajectory_valid(node,action,space_mask):
    vertices = get_vertices_for_curve(node,action,5)
    return sum([not is_obstacle(vertex,space_mask) for vertex in vertices[1:-1]])  == len(vertices[1:-1])# Dont need to check for first and last node, if all are true i,e all vertices are in free space, returns true, otherwise false
    

def a_star(start_position,end_position,delta_time,canvas_image):
    open_list = [(0, start_position)]
    heapq.heapify(open_list)
    space_mask = generate_space_map(canvas_image)
    visited_set = set()
    time_to_come_to_node = {start_position[:2]:0}
    node_list = {}   # {child(x,y) : parent(x,y,theta)}
    explored_nodes = {}
    path = []
    while open_list:
        _,node = heapq.heappop(open_list)
        if is_goal_node(node,end_position):
            print("Found path")
            path.append((node,"reached_goal"))
            parent = None
            while parent!= start_position:
                parent,action = node_list[node[:2]]
                path.append((parent,action))
                node = parent[:2]
            return path[::-1],explored_nodes
        action_node_pair = get_children(*node).items()
        visited_set.add(node[:2])
        explored_nodes[node] = []
        for action,child_node in action_node_pair:
            if child_node[:2] in visited_set or is_obstacle(node[:2],space_mask) or not check_trajectory_valid(node,action,space_mask):
                continue
            else:
                explored_nodes[node].append((action,child_node))
                time_to_come = time_to_come_to_node[node[:2]] + delta_time
                if time_to_come_to_node.get(child_node[:2],math.inf) > time_to_come:
                #if child_node[:2] not in time_to_come_to_node or time_to_come_to_node[child_node[:2]] > time_to_come:
                    time_to_come_to_node[child_node[:2]] = time_to_come
                    node_list[child_node[:2]] = node,action
                    total_time = time_to_come+ calculate_heuristic(child_node,end_position)
                    heapq.heappush(open_list,(total_time,child_node))
    print("No path found") 
    return None,None

def generate_space_map(canvas_image):
    space_mask = np.all(canvas_image.copy() == BACKGROUND_COLOR, axis=2)
    return space_mask
    
def get_path_falcon_simulation_path(path):
    arr = np.array(list(map(lambda x:x[0],path)))
    return np.diff(arr,axis=0)

def get_waypoints_for_ros(path):
    waypoints =  get_inverse_transformed_path(path)    
    return [(waypoint[0]/100,waypoint[1]/100,waypoint[2]) for waypoint in waypoints] # converting from cm to m

def get_inverse_transformed_path(path):
    return list(map(lambda x:inverse_transform_coordinate(x[0]),path))

def draw_curve(image,point,action,color):  
    vertices = get_vertices_for_curve(point,action)
    image = cv2.polylines(image,np.array([vertices],np.int32), False, color, 1)
    return image

def visualize(image,path,exploration_tree):
    iteration = max(int(len(exploration_tree) / (24 * 5)),1)
    frames = []
    for idx,parent in enumerate(exploration_tree):
        for child in exploration_tree[parent]:
            image = draw_curve(image,parent,child[0],((200,160,40)))
            cv2.circle(image,(int(child[1][0]),int(child[1][1])),1,(255,0,0),1)
        if idx % iteration == 0:
            frames.append(image.copy())
    
    for point,action in path:
        if action != "reached_goal":
            image = draw_curve(image,point,action,(0,0,250))
            frames.append(image.copy())
    return frames

def write_to_video(frames, name: str):
    if not name.endswith(".mp4"):
        name = name + ".mp4"
    flipped_frames = [cv2.flip(frame, 0) for frame in frames]
    clip = ImageSequenceClip(flipped_frames, fps=24)
    clip.write_videofile(name)
    print(f"Video saved as {name}")
                                   
def generate_map(clearance):
    # Scaling the robot_radius for now - else it's taking up the whole space
    clearance = clearance + ROBOT_RADIUS % 10
    
    print("\nGenerating the map....")
    
    # Create obstacle collection
    obstacle = ShapeCollection([])
    clearance_only = ShapeCollection([])

   # Add border as obstacle + clearance
    border_lines = [
        (Line(x - 0, "g"), Line(x - clearance, "l")),
        (Line(x - width + clearance, "g"), Line(x - width, "l")),
        (Line(y - 0, "g"), Line(y - clearance, "l")),
        (Line(y - height + clearance, "g"), Line(y - height, "l"))
    ]

    # Create each border shape and its clearance counterpart
    border1 = Shape([border_lines[0][0], border_lines[0][1], border_lines[2][0], border_lines[3][1]])
    border2 = Shape([border_lines[1][0], border_lines[1][1], border_lines[2][0], border_lines[3][1]])
    border3 = Shape([border_lines[2][0], border_lines[2][1], border_lines[0][0], border_lines[1][1]])
    border4 = Shape([border_lines[3][0], border_lines[3][1], border_lines[0][0], border_lines[1][1]])

    for border in [border1, border2, border3, border4]:
        obstacle.add_shape(border)

        # Inline clearance expansion
        clearance_lines = []
        for line in border.lines:
            if line.symbol == "g":
                offset_eq = line.equation + clearance
            else:
                offset_eq = line.equation - clearance
            clearance_lines.append(Line(offset_eq, line.symbol))

        clearance_border = Shape(clearance_lines)
        clearance_only.add_shape(clearance_border)

    # Add rectangles based on your sketch (converted mm to cm)
    def add_rectangle(x1, y1, w, h):
        lines = [
            Line(x - x1, "g"),
            Line(x - (x1 + w), "l"),
            Line(y - y1, "g"),
            Line(y - (y1 + h), "l")
        ]
        shape = Shape(lines)
        obstacle.add_shape(shape)
    
        clearance_lines = []
        for line in lines:
            if line.symbol == "g":
                offset_eq = line.equation + clearance
            else:
                offset_eq = line.equation - clearance
            clearance_lines.append(Line(offset_eq, line.symbol))

        clearance_shape = Shape(clearance_lines)
        clearance_only.add_shape(clearance_shape)
    
    obstacle_list = [
        (200, 0, 10, 200),
        (300, 100, 10, 200),
        (400, 0, 10, 100),
        (400, 200, 10, 100),
        (500, 0, 10, 200)
    ]

    for o in obstacle_list:
        add_rectangle(*o)

    # padding = 100  # in cm
    # canvas_width = width + 2 * padding
    
    # Draw the map
    canvas = np.full((height, width, 3), BACKGROUND_COLOR, dtype=np.uint8)
    for i in range(height):
        for j in range(width):
            if obstacle.check_point_inside_shape_collection([j, i]):
                canvas[i, j] = OBSTACLE_COLOR
            elif clearance_only.check_point_inside_shape_collection([j, i]):
                canvas[i, j] = CLEARANCE_COLOR  # ← CLEARANCE_COLOR (gray)    
                
    print("Press q to close the window and continue...")

    plt.title("Workspace Map")
    plt.imshow(canvas,origin="lower")
    plt.axis("off")
    plt.show()            

    return canvas

def ask_rpm():
    rpm1, rpm2 = map(int, input("\nEnter the 2 wheel RPM values in the form rpm1,rpm2: ").split(','))
    return(min(rpm1,rpm2),max(rpm1,rpm2))


def run_astar(start_position,end_position,clearance,robot_radius=ROBOT_RADIUS,wheel_radius=WHEEL_RADIUS,distance_between_wheels=WHEEL_DISTANCE,
              goal_threshold=DISTANCE_THRESHOLD,delta_time=DELTA_TIME,wheel_rpm_high=LOW_RPM,wheel_rpm_low=HIGH_RPM,visualization = False):
    global WHEEL_DIAMETER ,ROBOT_RADIUS ,WHEEL_DISTANCE,WHEEL_RADIUS ,LOW_RPM ,HIGH_RPM ,DISTANCE_THRESHOLD ,DELTA_TIME ,CLEARANCE, ASTAR_MAP
    ROBOT_RADIUS = robot_radius
    WHEEL_RADIUS = wheel_radius
    WHEEL_DISTANCE = distance_between_wheels
    CLEARANCE = clearance
    DISTANCE_THRESHOLD = goal_threshold
    DELTA_TIME = delta_time
    LOW_RPM = wheel_rpm_low
    HIGH_RPM = wheel_rpm_high
    if ASTAR_MAP is None:
        ASTAR_MAP = generate_map(clearance).copy()
    start = time.time()
    path,exploration_tree = a_star(start_position,end_position,delta_time,canvas_image=ASTAR_MAP)
    if path is not None and visualization:
        print("\nTotal time:",time.time()-start)
        print("Preparing visualization...")
        frames = visualize(ASTAR_MAP,path,exploration_tree)
        write_to_video(frames,"output.mp4")
    return path
        

def gather_inputs():
    clearance = ask_clearance()
    global ASTAR_MAP
    ASTAR_MAP = generate_map(clearance).copy()
    space_mask = generate_space_map(ASTAR_MAP)
    start_position = ask_position_to_user(space_mask, None,"start")
    end_position = ask_position_to_user(space_mask, None,"end") + (0,) # We dont take final goal orientation from user. Manually defining angle as 0 for consistency in shape in nodes
    low_rpm,high_rpm = ask_rpm()
    return start_position,end_position,low_rpm,high_rpm,clearance
    
def ask_clearance():
    try:
        clearance = int(input("Give a value for clearance with consideration of robot radius: "))
    except Exception as e:
        print("Error setting custom clearance!")
        clearance = CLEARANCE  
    return clearance

def ask_position_to_user(space_mask, position,location):
    if location == "start":
        message = "\nEnter the start coordinates in the form x,y,theta: "
    if location == "end":
        message = "\nEnter the goal coordinates in the form x,y: "
    while position is None:
        position = tuple(map(float, input(message).split(',')))
        position = transform_coordinate(position)
        if not is_obstacle(position,space_mask):
            return position
        else:
            print("\nInvalid position.") 
            position = None
   

if __name__ == "__main__":   
    start_position,end_position,low_rpm,high_rpm,clearance = gather_inputs()
    path = run_astar(start_position,end_position,clearance=clearance,wheel_rpm_low=low_rpm,wheel_rpm_high=high_rpm,visualization=True)
    transformed_path = get_inverse_transformed_path(path)
    print("Path:",transformed_path)