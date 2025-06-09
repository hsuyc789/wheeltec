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
tolerance_visual=20 #視覺對正的誤差範圍

# laser tracker parameter=====================================
Kp_laser=0.8
Ki_laser=0.002
Kd_laser=20 # 20
pe_laser=0.0
te_laser=0.0
e_laser=0.0
tolerance_laser=20 #0.5

visual_conf = 0.0

distance_cm = 0.0 #新增
ball_distance = 0.0
ball_offset = 0.0
goal_distance = 0.0
goal_offset = 0.0
trash_distance = 0.0
trash_offset = 0.0

# 加在此區域
ball_detected = 0.0
goal_detected = 0.0
trash_detected = 0.0
ball_confidence = 0.0
goal_confidence = 0.0
trash_confidence = 0.0


# filter
data_filter=[tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser,tolerance_laser]



# robot visual message callback
def visual_callback(data_sensor):
    global e_visual
    e_visual= ball_offset 
    # e_visual=data_sensor.data #e_visual是相機讀取的參數

# def visual_distance_callback(data_sensor): #新增距離接收
#     global distance_cm
#     distance_cm = data_sensor.data

def ball_distance_callback(msg):
    global ball_distance
    ball_distance = msg.data

def ball_offset_callback(msg):
    global ball_offset
    ball_offset = msg.data

def goal_distance_callback(msg):
    global goal_distance
    goal_distance = msg.data

def goal_offset_callback(msg):
    global goal_offset
    goal_offset = msg.data

def trash_distance_callback(msg):
    global trash_distance
    trash_distance = msg.data

def trash_offset_callback(msg):
    global trash_offset
    trash_offset = msg.data

def ball_detected_callback(msg):
    global ball_detected
    ball_detected = msg.data

def goal_detected_callback(msg):
    global goal_detected
    goal_detected = msg.data

def trash_detected_callback(msg):
    global trash_detected
    trash_detected = msg.data

def ball_confidence_callback(msg):
    global ball_confidence
    ball_confidence = msg.data

def goal_confidence_callback(msg):
    global goal_confidence
    goal_confidence = msg.data

def trash_confidence_callback(msg):
    global trash_confidence
    trash_confidence = msg.data


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
    # node.create_subscription(Sensordata, 'Visual_sensor_distance', visual_distance_callback, qos) #新增距離發包
    node.create_subscription(Sensordata, 'ball_distance', ball_distance_callback, qos)
    node.create_subscription(Sensordata, 'ball_offset', ball_offset_callback, qos)
    node.create_subscription(Sensordata, 'goal_distance', goal_distance_callback, qos)
    node.create_subscription(Sensordata, 'goal_offset', goal_offset_callback, qos)
    node.create_subscription(Sensordata, 'trash_distance', trash_distance_callback, qos)
    node.create_subscription(Sensordata, 'trash_offset', trash_offset_callback, qos)

    node.create_subscription(Sensordata, 'ball_detected', ball_detected_callback, qos)
    node.create_subscription(Sensordata, 'goal_detected', goal_detected_callback, qos)
    node.create_subscription(Sensordata, 'trash_detected', trash_detected_callback, qos)
    node.create_subscription(Sensordata, 'ball_confidence', ball_confidence_callback, qos)
    node.create_subscription(Sensordata, 'goal_confidence', goal_confidence_callback, qos)
    node.create_subscription(Sensordata, 'trash_confidence', trash_confidence_callback, qos)



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



    global count 
    count = 0

    while True:
        rclpy.spin_once(node)

        [uy, tey] = PID_controller(Kp_visual, Ki_visual, Kd_visual, ball_offset, pe_visual, te_visual)
        te_visual = tey
        pe_visual = ball_offset 

        if ball_distance <= 100: #設定最遠追蹤距離
            [ux, tex] = PID_controller(Kp_visual, Ki_visual, Kd_visual, ball_distance, pe_laser, te_laser)
            te_laser = tex
            pe_laser = ball_distance
        
        else:
            ux = 0.0  # 若雷射未進入指定距離，前進量設為 0
        
        rot = -2*uy

        # 直接將 uy 作為旋轉控制，取消 Y 平移控制
        # car_control(ux, 0.0, rot, pub_car)

        print("\n[📊 追蹤狀態]")
        print(f"🔵 Ball  ➤ 距離: {ball_distance:.2f} cm｜偏移: {ball_offset:.2f} px｜信心: {ball_confidence:.2f}｜偵測: {'✅' if ball_detected else '❌'}")
        print(f"🟡 Goal  ➤ 距離: {goal_distance:.2f} cm｜偏移: {goal_offset:.2f} px｜信心: {goal_confidence:.2f}｜偵測: {'✅' if goal_detected else '❌'}")
        print(f"🟢 Trash ➤ 距離: {trash_distance:.2f} cm｜偏移: {trash_offset:.2f} px｜信心: {trash_confidence:.2f}｜偵測: {'✅' if trash_detected else '❌'}")
        print(f"🔧 控制參數 ➤ 前進(ux): {-ux:.2f}｜旋轉(rz): {rot:.2f}")

        # print(f"ball_distance: {avg_filter(ball_distance)}, ux={-ux}, rz(rot)={rot}")
        # print(f"[🔵 Ball] 距離={ball_distance:.2f} cm，偏移={ball_offset:.2f} px")
        # print(f"[🟡 Goal] 距離={goal_distance:.2f} cm，偏移={goal_offset:.2f} px")
        # print(f"[🟢 Trash] 距離={trash_distance:.2f} cm，偏移={trash_offset:.2f} px")

        # if abs(ball_offset) < 5 and abs(avg_filter(ball_distance)) < 20:
        #     count += 1
        #     if count > 50:
        #         car_control(0.0, 0.0, 0.0, pub_car)
                
        # else:
        #     # if avg_filter(ball_distance) <= 20 or avg_filter(ball_distance) >= 100 or visual_conf < 0.6:
        #     if avg_filter(ball_distance) >= 100 or visual_conf < 0.6:
        #         car_control(0.0, 0.0, 0.0, pub_car)
        #     else:
        #         # 持續以 rz 控制角度修正
        #         car_control(ux, 0.0, rot, pub_car)
        #     count = 0

            #-ux為前進 rot旋轉

        if abs(ball_offset) > 3 and ball_detected == 1:
            car_control(0.0, 0.0, rot, pub_car)

        else :
            count += 1
            if count > 50:
                car_control(0.0, 0.0, 0.0, pub_car)
            
        count = 0


    print("[position tracker finished !!]")


if __name__ == '__main__':
    main() 