# -*- coding: utf-8 -*-

import numpy as np
from collections import deque
import json

# Function to get position of blank tile
def get_blank_tile_position(node:np.array):
  return np.where(node==0)

# Functions to move blank tile
def move_left(node:np.array):
  zero_position = get_blank_tile_position(node)
  if zero_position[1]!=0:
    node[zero_position[0],zero_position[1]],node[zero_position[0],zero_position[1]-1] = node[zero_position[0],zero_position[1]-1],node[zero_position[0],zero_position[1]]
    return True, node
  else:
    return False, node

def move_right(node:np.array):
  zero_position = get_blank_tile_position(node)
  if zero_position[1]!=2:
    node[zero_position[0],zero_position[1]],node[zero_position[0],zero_position[1]+1] = node[zero_position[0],zero_position[1]+1],node[zero_position[0],zero_position[1]]
    return True, node
  else:
    return False, node

def move_up(node:np.array):
  zero_position = get_blank_tile_position(node)
  if zero_position[0]!=0:
    node[zero_position[0],zero_position[1]],node[zero_position[0]-1,zero_position[1]] = node[zero_position[0]-1,zero_position[1]],node[zero_position[0],zero_position[1]]
    return True, node
  else:
    return False, node

def move_down(node:np.array):
  zero_position = get_blank_tile_position(node)
  if zero_position[0]!=2:
    node[zero_position[0],zero_position[1]],node[zero_position[0]+1,zero_position[1]] = node[zero_position[0]+1,zero_position[1]],node[zero_position[0],zero_position[1]]
    return True, node
  else:
    return False, node

# Function to expand node and get child nodes
def expand_node(node:np.array):
  node_list = []
  up_expansion = move_up(node.copy())
  left_expansion = move_left(node.copy())
  down_expansion = move_down(node.copy())
  right_expansion = move_right(node.copy())
  if up_expansion[0]:
    node_list.append(up_expansion[1])
  if left_expansion[0]:
    node_list.append(left_expansion[1])
  if down_expansion[0]:
    node_list.append(down_expansion[1])
  if right_expansion[0]:
    node_list.append(right_expansion[1])
  return node_list

def backtrack(node:np.array,child_parent_node,initial_node="NA"):
  node_list = []
  node = np.array2string(node,separator=",")
  while node != initial_node:
    node_list.append(node)
    node = child_parent_node[node]
  return node_list[::-1]

goal_node = np.array([[1,2,3],[4,5,6],[7,8,0]])
# Modify this array to desired initial mode row wise
# Example initial state :
# 8 6 7
# 2 5 4
# 3 0 1
initial_node = np.array([[8,6,7],[2,5,4],[3,0,1]])

# Dictionary to maintain parent child node relations {child_node:parent_node} , this also serves as data structure to store all nodes
child_parent_node = {np.array2string(initial_node,separator=","):np.array2string(initial_node,separator=",")}
unexpanded_nodes=deque([initial_node])
while len(unexpanded_nodes) > 0:
  # get the first node from unexpanded nodes
  current_node = unexpanded_nodes.popleft()
  # Check if the node is goal node
  if np.all(current_node == goal_node):
    # Backtrack and break the loop
    node_path = backtrack(current_node,child_parent_node,np.array2string(initial_node,separator=","))
    break
  else:
    # Get child nodes by trying all possible actions
    leaf_nodes = expand_node(current_node)
    # Loop through each child node
    for node in leaf_nodes:
      # Add node to unexpanded nodes only if it is unique. We dont want to add node which has already been expanded or is already present in queue of unexpanded nodes.
      if np.array2string(node,separator=",") not in child_parent_node:
        child_parent_node.update({np.array2string(node,separator=","):np.array2string(current_node,separator=",")})
        unexpanded_nodes.append(node)

node_list = {}
for i,node in enumerate(list(child_parent_node.keys())):
  node_list.update({node:i})

# Function to convert node in text format, ready to be written in text file
def convert_node_to_txt(node):
  return np.array2string(np.array(json.loads(node)).T.flatten()).lstrip("[").rstrip("]")

#Creating solution file
with open("nodePath.txt","w") as file:
  for n in node_path:
    file.write(convert_node_to_txt(n)+"\n")

# Creating file for all nodesexplored
with open("Nodes.txt","w") as file:
  for node in node_list:
    file.write(convert_node_to_txt(node)+"\n")

#Creating file consiting information about parent child node
with open("NodesInfo.txt","w") as file:
  file.write("Node_index Parent_Node_index Node"+"\n")
  for key,value in child_parent_node.items():
    file.write(str(node_list[key]) +" "+ str(node_list[value]) +" " + convert_node_to_txt(key)+"\n")

