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
from geometry_msgs.msg import Twist
from custom_msg.msg import Sensordata


# visual tracker parameter
Kp_visual=0.0004
Ki_visual=0.00000002
Kd_visual=0.01
pe_visual=0.0
te_visual=0.0
e_visual=0.0
tolerance_visual=20

visual_conf = 0.0

# robot visual message callback
def visual_callback(data_sensor):
    global e_visual
    e_visual=data_sensor.data

def visual_confidence_callback(data_sensor):
    global visual_conf
    visual_conf = data_sensor.data

# car control
def car_control(x,y,rz,pub):
    twist = Twist()
    twist.linear.x  = x; twist.linear.y = y;  twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = rz
    pub.publish(twist) 

# PID controller
def PID_controller(P,I,D,e,pe,te):
    u=P*e+I*(te+e)+D*(e-pe)
    return [u,te+e]

class Ball:
    def __init__(self):
        print("Create new tracking ball")
        self.ballHaveSeen = False
        self.position_x = 0
        self.width = 0
        self.distance = -1.0
        pass
    def calculate_distance(self):
        # Using linear function to calculate distance from x-width
        # y = Ax + B
        A = -2.0
        B = 3.0
        self.distance = A * self.width + B
        pass
    def process_position_x_callback(self, data):
        self.position_x = data.data
        if data.data == -1.0:
            self.ballHaveSeen = False
            self.distance = -1.0
        else:
            self.ballHaveSeen = True
    def process_width_callback(self, data):
        self.width = data.data

def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('task')
    ball = Ball()
    pub_car = node.create_publisher(Twist, 'cmd_vel', qos)
    node.create_subscription(Sensordata,'Visual_sensor_vel',visual_callback,qos)
    node.create_subscription(Sensordata, 'Visual_sensor_confidence',visual_confidence_callback, qos )
    node.create_subscription(Sensordata, 'Visual_ball_position_x', ball.process_position_x_callback, qos )
    node.create_subscription(Sensordata, 'Visual_ball_width', ball.process_width_callback, qos )
    rclpy.spin_once(node)
    print("[node create !!]")

    # Parameter
    global pe_visual
    global te_visual
    global e_visual
    global Kp_visual
    global Ki_visual
    global Kd_visual
    global tolerance_visual


    # step 1 :calibration Y-axis
    count=0
    while (True):
        rclpy.spin_once(node)
        [u,te]=PID_controller(Kp_visual,Ki_visual,Kd_visual,e_visual,pe_visual,te_visual)
        te_visual=te
        pe_visual=e_visual
        car_control(0.0,u,0.0,pub_car)

        if abs(e_visual)<tolerance_visual:
            count=count+1
            if (count>50):
                car_control(0.0,0.0,0.0,pub_car)
                break
        else:
            count=0
    print("[Visual tracker finished !!]")

    # step 3 :calibration XY-axis
    tolerance_laser=0.015
    count=0
    while (True):
        rclpy.spin_once(node)
        [uy,tey]=PID_controller(Kp_visual,Ki_visual,Kd_visual,e_visual,pe_visual,te_visual)
        te_visual=tey
        pe_visual=e_visual
        if(e_laser <= 1.1):
            [ux,tex]=PID_controller(Kp_laser,Ki_laser,Kd_laser,e_laser,pe_laser,te_laser)
            te_laser=tex
            pe_laser=e_laser
        else:
            pass
        
        # car_control(-1*ux,uy,0.0,pub_car)
        
        # print(abs(e_visual))
        # print(visual_conf)
        print(f"e_laser: {avg_filter(e_laser)},ux={-ux}, uy={uy}")
        
        
        # print(e_visual)

        if (abs(e_visual)<tolerance_visual)and(abs(avg_filter(e_laser))<tolerance_laser):
            count=count+1
            if (count>50):
                car_control(0.0,0.0,0.0,pub_car)
                # break
        else:
            if avg_filter(e_laser)<=-1.7 or avg_filter(e_laser)>=1.1 or visual_conf < 0.85:
                # print("stop")
                car_control(0.0,0.0,0.0,pub_car)
            else:
                rot = np.arctan(uy / (-ux))
                print(f"rot={rot}")
                car_control(-ux,0.0,rot,pub_car)
                pass
            count=0
            
       
        # print(f"{-1 * ux}, {uy}")
    print("[position  tracker finished !!]")



if __name__ == '__main__':
    main()
