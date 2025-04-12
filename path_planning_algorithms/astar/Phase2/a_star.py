import time
import math
from functools import cache
import heapq
import numpy as np
from moviepy import ImageSequenceClip
import cv2
import matplotlib.pyplot as plt

wheel_diameter = 6.6 # in cm
robot_radius = 22.0 # in cm
wheel_distance = 28.7 # in cm
wheel_radius = wheel_diameter/2
LOW_RPM = 50 # default radian/s
HIGH_RPM = 100 # default radian/s
DISTANCE_THRESHOLD = 2 # cm
DELTA_TIME = 1
BACKGROUND_COLOR = (232,215,241)
OBSTACLE_COLOR = (0,0,0)

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

@cache
def calculate_wheel_velocity(rpm, radius = wheel_radius):
  return rpm * 2 * math.pi * radius / 60

@cache
def calculate_velocity(left_rpm,right_rpm):
  left_wheel_velocity = calculate_wheel_velocity(left_rpm) # mm / sec
  right_wheel_velocity = calculate_wheel_velocity(right_rpm)
  linear_velocity =  (left_wheel_velocity + right_wheel_velocity) / 2
  angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_distance
  if left_rpm == right_rpm:
    icc_radius = None
  else:
    icc_radius = ( wheel_distance * linear_velocity )/(right_wheel_velocity - left_wheel_velocity)
  return {"linear":linear_velocity,"angular":angular_velocity,"icc_radius":icc_radius}

@cache
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
        # if theta_new >= math.pi :
        #     theta_new = -2*math.pi + theta_new
        # elif theta_new < -math.pi:
        #     theta_new = 2*math.pi + theta_new
    return x_new,y_new,theta_new

def get_children(x,y,theta):
    return {action:get_next_position(x,y,theta,*action_list[action]) for action in action_list.keys()}

def calculate_heuristic(point1,point2):
    return math.hypot(point1[0] - point2[0],point1[1] - point2[1]) / calculate_wheel_velocity(HIGH_RPM)

# Implement this
def is_obstacle(point,space_mask):
    x , y = int(point[0]) , int(point[1])
    h,w = space_mask.shape
    return  x < 0 or x >= w or y < 0 or y >= h or not space_mask[y][x]


def is_goal_node(node,goal_node,threshold = DISTANCE_THRESHOLD):
    return math.hypot(node[0] - goal_node[0],node[1] - goal_node[1]) < threshold

@cache
def get_vertices_for_curve(point1,action,resolution=10):
    x,y,theta = point1
    points = [(x,y)]
    for _ in range(resolution):
        x,y,theta = get_next_position(x,y,theta,*action_list[action],DELTA_TIME/resolution)
        points.append((x,y))
    return points

@cache
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
                if child_node in time_to_come_to_node:
                    if time_to_come_to_node[child_node[:2]] < time_to_come:
                        time_to_come_to_node[child_node[:2]] = time_to_come
                        node_list[child_node[:2]] = node,action
                else:
                    time_to_come_to_node[child_node[:2]] = time_to_come
                    node_list[child_node[:2]] = node,action
                total_time = time_to_come_to_node[child_node[:2]] + calculate_heuristic(child_node,end_position)
                heapq.heappush(open_list,(total_time,child_node))
    print("No path found") 
    return None,None

def generate_space_map(canvas_image):
    space_mask = np.all(canvas_image.copy() == BACKGROUND_COLOR, axis=2)
    return space_mask
    
def get_path_falcon_simulation_path(path):
    arr = np.array(list(map(lambda x:x[0],path)))
    return np.diff(arr,axis=0)

def get_velocities_for_ros(path):
    actions =  list(map(lambda x:x[1],path[:-1]))
    vel = []
    for action in actions:
        velocity = calculate_velocity(*action_list[action])
        velocity["linear"] = velocity["linear"] / 100  # Coverting cm/s to m/s
        velocity.pop("icc_radius")
        vel.append(velocity)       
    return vel  

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

def write_to_video(frames,name:str):
    if not name.endswith(".mp4"):
        name = name + ".mp4"
    clip = ImageSequenceClip(frames, fps=24)
    clip.write_videofile(name)
    print(f"Video saved as {name}")
                                   
def generate_map(clearance):
    image = np.full(shape=(600,540,3),fill_value=BACKGROUND_COLOR,dtype=np.uint8)
    image[29:300,20:200] = OBSTACLE_COLOR  # [ ylimit, xlimit]
    return image

def ask_rpm():
    rpm1= int(input(f"Enter value for RPM 1:"))
    rpm2= int(input(f"Enter value for RPM 2:"))
    return(min(rpm1,rpm2),max(rpm1,rpm2))

def run_astar():
    astar_map = generate_map(clearance=2)
    space_mask = generate_space_map(astar_map)
    start_position = ask_position_to_user(space_mask, None,"start")
    end_position = ask_position_to_user(space_mask, None,"end") + (0,) # We dont take final goal orientation from user. Manually defining angle as 0 for consistency in shape in nodes
    delta_time = DELTA_TIME
    global LOW_RPM,HIGH_RPM 
    LOW_RPM,HIGH_RPM = ask_rpm()
    start = time.time()
    path,exploration_tree = a_star(start_position,end_position,delta_time,canvas_image=astar_map)
    if path is not None:
        print("Total time:",time.time()-start)
        print("Preparing visualization...")
        frames = visualize(astar_map,path,exploration_tree)
        write_to_video(frames,"sample.mp4")
    

def ask_position_to_user(space_mask, position,location):
    if location == "start":
        size = 3
        message = "Enter x(cm),y(cm),theta(radians) for start position:"
    if location == "end":
        size = 2
        message = "Enter x(cm),y(cm) for end position:"
    while position is None:
        position = tuple(map(int, input(message).split(',')))
        if not is_obstacle(position,space_mask) and len(position) == size:
            return position
        else:
            position = None
    
    

if __name__ == "__main__":
    run_astar()
    # start = time.time()
    # start_position = 4,4,math.pi/2
    # end_position = 330,490,0
    # image = generate_map(clearance=2)
    # path,exploration_tree = a_star(start_position,end_position,DELTA_TIME,canvas_image=image)
    # if path is not None:
    #     print("Total time:",time.time()-start)
    #     print("Preparing visualization...")
    #     frames = visualize(image,path,exploration_tree)
    #     write_to_video(frames,"sample.mp4")
    