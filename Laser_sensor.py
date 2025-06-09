#=====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
#=====================================================

#!/usr/bin/env python3
# coding=utf-8

# Import necessary modules for ROS 2 communication
import os  # Operating system module
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module
import rclpy  # ROS 2 client library
import time
import math
import cv2
import cv_bridge
import numpy as np
import numpy
import json

# Import message types and Quality of Service (QoS) profile for ROS 2
from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from custom_msg.msg import Sensordata
from geometry_msgs.msg import Twist

# Parameter X
x_target = 1 #===========================================================
   
# lidar data callback
def lidar_callback(msg, pub):
    global x_target
    
    # Get the front distance
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    ranges = msg.ranges
    
    # distance from -5~5(deg)
    dist_array=[x_target*2]
    count=0
    for i in range( -3, 3, 1):
        dist = ranges[i]
        if math.isinf(dist):
            count=count+1
        elif (dist<(2*x_target)):
            dist_array.append(dist)
            
    if count>5 :
        dist = 0.2
    else:
        dist = min(dist_array)
    
    print(dist)
    sensor_msg=Sensordata()
    sensor_msg.data=float(x_target-dist)
    pub.publish(sensor_msg)
    
        
    
    
def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('Laser_sensor')
    pub = node.create_publisher(Sensordata, 'Laser_sensor_vel', qos)
    lidar_sub = node.create_subscription(LaserScan,'scan',lambda msg: lidar_callback(msg, pub), qos)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
