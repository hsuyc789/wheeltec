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
from geometry_msgs.msg import PoseWithCovarianceStamped


time_kick = 0

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

ball_confidence = 0.0
ball_detected = 0.0
goal_confidence = 0.0
goal_detected = 0.0
trash_confidence = 0.0
trash_detected = 0.0

pe_visual_x = 0.0
te_visual_x = 0.0
pe_visual_y = 0.0
te_visual_y = 0.0
pe_visual_rx = 0.0
te_visual_rx = 0.0

# 建圖位置
robot_position = [0.0, 0.0]
robot_yaw = 0.0
# current_robot_pos_x = 0.0
# current_robot_pos_y = 0.0

# 在尋找球時，要在球場上以之字形前進，第一個點會到球場左後方，之後沿右前、左前方向逐步移動到終點
# 如果在中途找到球，則直接中斷此尋找進程
# 要移動到的點 (x(直向), y(橫向))，x 從 0 由後往前遞增； y 從 0 由左往右遞增:
SEARCH_REACH_POINTS = [[-4.25, 0.25], [-3.85, -1.00], [-3.45, 0.80], [-3.05, -1.00], [-2.65, 0.80], [-2.25, -1.00]] #向左y正 向前x正
GOAL_POINTS = [[0.18, 0.39], [0.18, 0.15], [0.18, -0.15], [0.18, -0.34]]
# 校正 ball distance
# BALL_DISTANCE_OFFSET 數值定義 ball 距離為多少時被定義成 0
BALL_DISTANCE_OFFSET = 20.0

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
    ball_distance = msg.data - BALL_DISTANCE_OFFSET

def ball_offset_callback(msg):
    global ball_offset
    ball_offset = msg.data
    
def ball_detected_callback(msg):
    global ball_detected
    ball_detected = msg.data

def goal_distance_callback(msg):
    global goal_distance
    goal_distance = msg.data

def goal_offset_callback(msg):
    global goal_offset
    goal_offset = msg.data
    
def goal_detected_callback(msg):
    global goal_detected
    goal_detected = msg.data

def trash_distance_callback(msg):
    global trash_distance
    trash_distance = msg.data

def trash_offset_callback(msg):
    global trash_offset
    trash_offset = msg.data

def trash_detected_callback(msg):
    global trash_detected
    trash_detected = msg.data

def visual_confidence_callback(data_sensor):
    global visual_conf
    visual_conf = data_sensor.data 


# robot laser message callback
def laser_callback(data_sensor):
    global e_laser
    e_laser=data_sensor.data #e_laser是雷達讀取的參數
    
# 時間等待
def wait_duration(node, duration_sec):
    start_time = time.time()
    while time.time() - start_time < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

# 目前位置讀取
def position_callback(msg):
    global robot_position, robot_yaw
    
    # x, y 位置
    robot_position[0] = msg.pose.pose.position.x
    robot_position[1] = msg.pose.pose.position.y

    # 四元數+轉換
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    
    # robot角度
    robot_yaw = math.atan2(siny_cosp, cosy_cosp)

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

# GOAL_POINTS = [[0, 0], [4, 10], [8, 0], [12, 10], [16, 0], [20, 0]]
# 射門軌跡校正
# def shoot_trajectory_position(current_position, current_yaw, goal_point, pub_car, node):
#     global pe_visual_x, te_visual_x, pe_visual_y, te_visual_y, pe_visual_rx, te_visual_rx
    
#     x, y = current_position
#     goal_x, goal_y = goal_point  # 目前先取第一個點
#     ball_position = [x+0.2, y]  # 假設球在機器人前方 20cm
#     print(goal_point)
#     print(current_position)

#     # 球到目標的方向向量
#     vec_goal = [goal_x - ball_position[0], goal_y - ball_position[1]]
#     # 單位化
#     length = math.hypot(vec_goal[0], vec_goal[1])
#     vec_goal_norm = [vec_goal[0] / length, vec_goal[1] / length]

#     # 機器人切入點
#     target_position = [
#         ball_position[0] - vec_goal_norm[0],
#         ball_position[1] - vec_goal_norm[1]
#     ]

#     # 朝向目標的角度（從球指向球門）
#     target_yaw = math.atan2(vec_goal[1], vec_goal[0])
    
#     Kr = 1
#     Km = 1
#     r = Kr * (target_yaw - current_yaw)
#     x = Km * (target_position[0] - current_position[0])
#     y = Km * (target_position[1] - current_position[1])
    
#     [ux, tex] = PID_controller(Kp_visual, Ki_visual, Kd_laser, x, pe_visual_x, te_visual_x)
#     te_visual_x = tex
#     pe_visual_x = x
#     [uy, tex] = PID_controller(Kp_visual, Ki_visual, Kd_laser, y, pe_visual_y, te_visual_y)
#     te_visual_y = tex
#     pe_visual_y = y
#     [rx, tex] = PID_controller(Kp_visual, Ki_visual, Kd_laser, r, pe_visual_rx, te_visual_rx)
#     te_visual_rx = tex
#     pe_visual_rx = r
#     car_control(ux, -uy, rx, pub_car)

#     return target_position, target_yaw

#  確認球是否在踢球範圍內
def ball_in_range(dx, dy, detected):
    X_MIN = -5
    X_MAX = 25
    Y_MIN = -5
    Y_MAX = 5
    if detected == 0:
        print("Ball not detected")
        return False
    else:
        return (X_MIN <= abs(dx) <= X_MAX) and (Y_MIN <= dy <= Y_MAX)
    
def robot_in_pos(cur_x, cur_y, target_x, target_y):
    # 確認車子是否在目標座標的範圍內
    return (target_x - 0.5 < cur_x < target_x + 0.5) and (target_y - 0.5 < cur_y < target_y + 0.5)

def kick_ball(current_position, current_yaw, trash_offset, pub_car):# trash_y_error
    # global pe_visual_x, te_visual_x, pe_visual_y, te_visual_y, pe_visual_rx, te_visual_rx
    # for goal_point in GOAL_POINTS:
        # print("踢球軌跡校正中")
        # target_pos, target_yaw = shoot_trajectory_position(current_position, current_yaw, goal_point, pub_car, node)
        # wait_duration(node, 5.0)  # 給時間讓機器人移動

        # 判斷是否可以踢
    print(f"trash_detected = {trash_detected}, trash_offset = {trash_offset}")
    if trash_detected and abs(trash_offset) <= 50:
        # 微調旋轉角度對準垃圾
        car_control(0.0, 0.0, 0.05, pub_car)
        if abs(ball_offset) > 15:
            car_control(0.0, 0.0, 0.0, pub_car)
        return False
    else:
        # 判斷為時機已到，直接踢
        print("踢球!!")
        car_control(2.0, 0.0, 0.0, pub_car)
        time.sleep(2.0)
        car_control(0.0, 0.0, 0.0, pub_car)
        time.sleep(2.0)
        return True
            
        
    # 踢球的程式
    
def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('task')
    pub_car = node.create_publisher(Twist, 'cmd_vel', qos)
    node.create_subscription(Sensordata,'Visual_sensor_vel',visual_callback,qos)
    node.create_subscription(Sensordata,'Laser_sensor_vel',laser_callback,qos)
    #node.create_subscription(Sensordata, 'Visual_sensor_confidence',visual_confidence_callback, qos )
    #node.create_subscription(Sensordata, 'Visual_sensor_distance', visual_distance_callback, qos) #新增距離發包
    node.create_subscription(Sensordata, 'ball_distance', ball_distance_callback, qos)
    node.create_subscription(Sensordata, 'ball_offset', ball_offset_callback, qos)
    node.create_subscription(Sensordata,'ball_detected',ball_detected_callback,qos)
    node.create_subscription(Sensordata, 'goal_distance', goal_distance_callback, qos)
    node.create_subscription(Sensordata, 'goal_offset', goal_offset_callback, qos)
    node.create_subscription(Sensordata,'goal_detected',goal_detected_callback,qos)
    node.create_subscription(Sensordata, 'trash_distance', trash_distance_callback, qos)
    node.create_subscription(Sensordata, 'trash_offset', trash_offset_callback, qos)
    node.create_subscription(Sensordata,'trash_detected',trash_detected_callback,qos)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', position_callback, qos)


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
    
    global pe_visual_x, te_visual_x, pe_visual_y, te_visual_y, pe_visual_rx, te_visual_rx
    
    # global current_robot_pos_x
    # global current_robot_pos_y

    global count 
    count = 0
    
    ball_in_range_count = 0
    now_search_step = 0
    max_search_step = len(SEARCH_REACH_POINTS)
    
    is_kicking_ball = False # 紀錄是否正在踢球

    while True:
        ux = 0.0
        uy = 0.0
        
        rclpy.spin_once(node)
        #shoot_trajectory_position(robot_position, robot_yaw, pub_car)
        ball_is_in_range = ball_in_range(ball_distance, ball_offset, ball_detected)
        
        # 計算移動速度(ux, uy)
        if is_kicking_ball:
            print("踢球模式")
            kicked = kick_ball(robot_position, robot_yaw, trash_offset, pub_car)
            if kicked:
                is_kicking_ball = False
                ball_in_range_count = 0
            pass
        elif ball_is_in_range: # 如果球有在踢球範圍
            print("對準完成，進入踢球模式")
            now_search_step = 0
            # todo: 準備開始踢球程式
            # 數到 50
            ball_in_range_count += 1
            if ball_in_range_count > 15:
                # 開始踢球
                ux = 0
                uy = 0
                is_kicking_ball = True
                pass
            pass
        elif ball_detected: # 如果有看到球，但不在踢球範圍內
            print("看到球了 對準球中")
            # todo: 校正機器人位置到踢球範圍內
            # now_search_step = 0
            is_kicking_ball = False
            # 校正 x 位置
            [ux, tex] = PID_controller(Kp_visual, Ki_visual, Kd_laser, ball_distance, pe_visual_x, te_visual_x)
            te_visual_x = tex
            pe_visual_x = ball_distance
            # 校正 y 位置
            [uy, tey] = PID_controller(Kp_visual, Ki_visual, Kd_visual, ball_offset, pe_visual_y, te_visual_y)
            te_visual_y = tey
            pe_visual_y = ball_offset
            car_control(ux, -uy, 0.0, pub_car)
            
            ball_in_range_count = 0
        else: 
            #print("沒有看到球 魚骨挖球中")
            print("沒有看到球 後退找")
            #如果都沒有看到球
            #todo: 掃描球(魚骨挖礦法)
            # ball_in_range_count = 0
            # is_kicking_ball = False
            # if now_search_step >= max_search_step:
            #     now_search_step = 0
            # if robot_in_pos(robot_position[0], robot_position[1], SEARCH_REACH_POINTS[now_search_step][0], SEARCH_REACH_POINTS[now_search_step][1]):
            #     # 如果車子尋找時走到自己定義的中途點
            #     now_search_step += 1
            # # 計算當前要走到中途點的速度
            # ku = 1 # 速度調整的常數
            # x = ku * (SEARCH_REACH_POINTS[now_search_step][0] - robot_position[0]) #ux
            # y = ku * (SEARCH_REACH_POINTS[now_search_step][1] - robot_position[1]) #uy
            # [ux, tex] = PID_controller(Kp_visual, Ki_visual, Kd_visual, x, pe_visual_x, te_visual_x)
            # te_visual_x = tex
            # pe_visual_x = ux
            # [uy, tey] =  PID_controller(Kp_visual, Ki_visual, Kd_visual, y, pe_visual_y, te_visual_y)
            # te_visual_y = tey
            # pe_visual_y = uy
            # car_control(ux, -uy, 0.0, pub_car)
            car_control(-0.05, 0.0, 0.0, pub_car)
            pass
            
        
        # rot = -2*uy

        # 直接將 uy 作為旋轉控制，取消 Y 平移控制
        # car_control(ux, 0.0, rot, pub_car)

        print("\n[📊 追蹤狀態]")
        print(f"🔵 Ball  ➤ 距離: {ball_distance:.2f} cm｜偏移: {ball_offset:.2f} px｜信心: {ball_confidence:.2f}｜偵測: {'✅' if ball_detected else '❌'}")
        print(f"🟡 Goal  ➤ 距離: {goal_distance:.2f} cm｜偏移: {goal_offset:.2f} px｜信心: {goal_confidence:.2f}｜偵測: {'✅' if goal_detected else '❌'}")
        print(f"🟢 Trash ➤ 距離: {trash_distance:.2f} cm｜偏移: {trash_offset:.2f} px｜信心: {trash_confidence:.2f}｜偵測: {'✅' if trash_detected else '❌'}")
        # print(f"🔧 控制參數 ➤ 前進(ux): {-ux:.2f}｜左右(uy): {uy:.2f}｜旋轉(rz): {rot:.2f}")
        
        # car_control(ux, uy, 0.0, pub_car)
        
            
if __name__ == '__main__':
    main()