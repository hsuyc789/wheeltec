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
tolerance_visual=10 #視覺對正的誤差範圍

# laser tracker parameter=====================================
Kp_laser=0.8
Ki_laser=0.002
Kd_laser=20 # 20
pe_laser=0.0
te_laser=0.0
e_laser=0.0
tolerance_laser=0.5 #0.5

visual_conf = 0.0
distance_cm = 0.0


# filter
data_filter=[tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser]



# robot visual message callback
def visual_callback(data_sensor):
    global e_visual
    e_visual=data_sensor.data #e_visual是相機讀取的參數

def visual_distance_callback(data_sensor): #新增距離接收
    global distance_cm
    distance_cm = data_sensor.data

def visual_confidence_callback(data_sensor):
    global visual_conf
    visual_conf = data_sensor.data 


# robot laser message callback
def laser_callback(data_sensor):
    global e_laser
    e_laser=data_sensor.data #e_laser是雷達讀取的參數


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


# Average filter
def avg_filter(data):
    global data_filter
    for i in range(0,9,1):
        data_filter[i]=data_filter[i+1]
    data_filter[9]=data
    return sum(data_filter)/10.0


def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('task')
    pub_car = node.create_publisher(Twist, 'cmd_vel', qos)
    node.create_subscription(Sensordata,'Visual_sensor_vel',visual_callback,qos)
    node.create_subscription(Sensordata,'Laser_sensor_vel',laser_callback,qos)
    node.create_subscription(Sensordata, 'Visual_sensor_confidence',visual_confidence_callback, qos )
    node.create_subscription(Sensordata, 'Visual_sensor_distance', visual_distance_callback, qos) #新增距離發包

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

    global pe_laser
    global te_laser
    global e_laser
    global Kp_laser
    global Ki_laser
    global Kd_laser
    global tolerance_laser


    # step 1 :calibration Y-axis
    # count=0
    # while (True):
    #     rclpy.spin_once(node)
    #     [u,te]=PID_controller(Kp_visual,Ki_visual,Kd_visual,e_visual,pe_visual,te_visual)
    #     te_visual=te
    #     pe_visual=e_visual
    #     car_control( 0.0, u, 0.0, pub_car)

    #     if abs(e_visual)<tolerance_visual:
    #         count=count+1
    #         if (count>50):
    #             car_control( 0.0, 0.0, 0.0, pub_car)
    #             break
    #     else:
    #         count=0
    # print("[Visual tracker finished !!]")

    # # step 2 :calibration X-axis
    # count=0
    # while (True):
    #     rclpy.spin_once(node)
    #     [u,te]=PID_controller(Kp_laser,Ki_laser,Kd_laser,e_laser,pe_laser,te_laser)
    #     te_laser=te
    #     pe_laser=e_laser
        
    #     #print((avg_filter(e_laser))) #02

    #     if abs(avg_filter(e_laser))<tolerance_laser:
    #         count=count+1
    #         if (count>50):
    #             car_control( 0.0, 0.0, 0.0, pub_car)
    #             break
    #     else:
    #         if avg_filter(e_laser)<=-1.7 or avg_filter(e_laser)>=1.5:
    #             # print("stop")
    #             car_control( 0.0, 0.0, 0.0, pub_car)
    #         else:
    #             car_control( -1*u, 0.0, 0.0, pub_car)
    #             pass
    #         count=0
    # print("[Laser tracker finished !!]")

    # step 3 :calibration XY-axis
    #tolerance_laser=0.015 #02
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
        
        car_control(-1*ux,uy,0.0,pub_car) #01
        
        #print(abs(e_visual)) #01
        #print(visual_conf) #01
        print(f"e_laser: {avg_filter(e_laser)},ux={-ux}, uy={uy}")
        
        print(f"e_laser={e_laser}") #01
        print(f"e_visual={e_visual}") #01
        print(f"[📏 相機估測距離] {distance_cm:.2f} cm") #新增


        if (abs(e_visual)<tolerance_visual)and(abs(avg_filter(e_laser))<tolerance_laser):
            count=count+1
            if (count>50):
                car_control( 0.0, 0.0, 0.0, pub_car)
                # break
        else:
            if avg_filter(e_laser)<=-1.7 or avg_filter(e_laser)>=1.1 or visual_conf < 0.85:
                # print("stop")
                car_control( 0.0, 0.0, 0.0, pub_car)
            else:
                #rot = np.arctan(uy / (-ux))
                rot = uy
                print(f"rot={rot}")
                car_control( -ux, 0.0, rot, pub_car)
                pass
            count=0
            
       
        # print(f"{-1 * ux}, {uy}")
    print("[position  tracker finished !!]")



if __name__ == '__main__':
    main()
