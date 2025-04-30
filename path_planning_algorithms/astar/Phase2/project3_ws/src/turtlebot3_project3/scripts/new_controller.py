#!/usr/bin/env python3

import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from controller import run_astar,gather_inputs,get_waypoints_for_ros

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel_unstamped', 10) 
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.velocity_msg = Twist()
        start_position, end_position, low_rpm, high_rpm, clearance = gather_inputs()
        waypoints = list(run_astar(start_position,end_position,clearance=clearance,wheel_rpm_low=low_rpm,wheel_rpm_high=high_rpm,visualization=False))
        path = list(get_waypoints_for_ros(waypoints)) + [None]
        self.path = deque(path)
        self.target_node = self.path.popleft()
        self.start_time = time.time()
        self.total_distance_error = 0
        self.total_angular_error = 0
        self.max_lin_vel_reached = -math.inf
        self.max_ang_vel_reached = -math.inf
        
    def odom_callback(self,message: Odometry):
        pose = message.pose.pose
        vel = message.twist.twist
        measured_x = pose.position.x
        measured_y = pose.position.y
        measured_roll,measured_pitch, measured_yaw = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]) 
        measured_linear_velocity = vel.linear.x
        measured_angular_velocity = vel.angular.z
        self.max_lin_vel_reached= max(self.max_lin_vel_reached,measured_linear_velocity)
        self.max_ang_vel_reached= max(self.max_ang_vel_reached,measured_angular_velocity)
        self.get_logger().info(f"Max lin velocity {self.max_lin_vel_reached} Max ang {self.max_ang_vel_reached}")
        linear_velocity, angular_velocity = self.controller(measured_x,measured_y,measured_yaw,measured_linear_velocity)
        self.publish_velocity(linear_velocity,angular_velocity)
        
    def controller(self,measured_x:float,measured_y:float,measured_yaw:float,measured_lin_vel:float):
        if self.target_node is not None:
            time_now = time.time()
            dt = time_now - self.start_time 
            self.start_time = time_now
            target_x,target_y,target_yaw = self.target_node
            distance_error = math.hypot(target_x-measured_x,target_y-measured_y)
            angular_error = math.atan2(target_y-measured_y,target_x-measured_x) - measured_yaw
            if angular_error > math.pi:
                angular_error-= 2*math.pi
            elif angular_error <= -math.pi:
                angular_error+= 2*math.pi
            self.get_logger().info(f"Distance error {distance_error} Angular error {angular_error}")
            
            if distance_error < 0.1 :
                self.target_node = self.path.popleft()
                self.total_distance_error = 0
                self.total_angular_error = 0
                self.get_logger().info(f"Target node set to : {self.target_node}")      
            self.total_distance_error += distance_error*dt
            self.total_angular_error += angular_error*dt            
            proportional_linear_k = 0.3
            proportioanl_angular_k = 0.4
            integral_linear_k = 0#0.1
            integral_angular_k = 0#0.2
            alpha = 1.0
            linear_vel = measured_lin_vel*(1-alpha) + alpha*(proportional_linear_k*distance_error + integral_linear_k*self.total_distance_error)
            angular_vel = proportioanl_angular_k * angular_error + integral_angular_k*self.total_angular_error
            return 9*linear_vel,9*angular_vel
        else:
            return 0.0,0.0
        
    def publish_velocity(self,linear_velocity:float,angular_velocity:float):
        self.velocity_msg.linear.x = linear_velocity
        self.velocity_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(self.velocity_msg)
        

if __name__ == '__main__':
    rclpy.init()    
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()