# =====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
# =====================================================

#!/usr/bin/env python3
# coding=utf-8
# －－－ 新增／保留的共用 import －－－
import rclpy, math, time
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg     import Odometry            # ★ 新增
from custom_msg.msg   import Sensordata
# （OpenCV、numpy… 仍可保留，如果後面沒用可刪）

# ───────── 全域參數 ──────────
# PID 與偵測相關（照舊）
Kp_visual, Ki_visual, Kd_visual = 20e-4, 2e-8, 10 # 4e-4  2e-8  0.01
Kp_laser , Ki_laser , Kd_laser  = 0.8 , 0.002, 20
tolerance_visual, tolerance_laser = 20, 20

# 場地左右邊界（以 odom frame Y 軸為準；單位 m）
Y_MIN, Y_MAX = -70, 70     #cm 球門寬100cm
MARGIN       = 0.10     # 與牆保留 10 cm
SEARCH_V     = 0.2     # 橫移搜尋速度 (m/s)

# －－－ 里程計姿態（全域）－－－
robot_x = robot_y = robot_yaw = 0.0   # ★ 新增

# ===== 回呼函式 =====
def odom_cb(msg):                     # ★ 新增
    """把 /odom 轉成 (x,y,yaw) 全域變數"""
    global robot_x, robot_y, robot_yaw
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    robot_yaw = math.atan2(2*(q.w*q.z), 1 - 2*q.z*q.z)

def ball_distance_cb(m):   globals()['ball_distance']  = m.data
def ball_offset_cb(m):     globals()['ball_offset']    = m.data
def ball_detected_cb(m):   globals()['ball_detected']  = m.data
def goal_distance_cb(m):   globals()['goal_distance']  = m.data
def goal_offset_cb(m):     globals()['goal_offset']    = m.data
def goal_detected_cb(m):   globals()['goal_detected']  = m.data
def trash_distance_cb(m):  globals()['trash_distance'] = m.data
def trash_offset_cb(m):    globals()['trash_offset']   = m.data
def trash_detected_cb(m):  globals()['trash_detected'] = m.data

def ball_confidence_cb(m):   globals()['ball_confidence']  = m.data
def goal_confidence_cb(m):   globals()['goal_confidence']  = m.data
def trash_confidence_cb(m):  globals()['trash_confidence'] = m.data


# ====== 車速封裝 ======
def car_control(x, y, rz, pub):
    t = Twist()
    t.linear.x,  t.linear.y,  t.linear.z  = x, y, 0.0
    t.angular.x, t.angular.y, t.angular.z = 0.0, 0.0, rz
    pub.publish(t)

def PID(P, I, D, e, pe, ie):
    u = P*e + I*(ie + e) + D*(e-pe)
    return u, ie + e
ball_confidence = goal_confidence = trash_confidence = 0.0

# ====================== main ======================
def main():
    global ball_distance, ball_offset, ball_detected
    global goal_distance, goal_offset, goal_detected
    global trash_distance, trash_offset, trash_detected
    ball_distance = ball_offset = goal_distance = goal_offset = trash_distance = trash_offset = 0.0
    ball_detected = goal_detected = trash_detected = 0

    # ─── ROS 2 節點與 Topic ───
    rclpy.init()
    qos  = QoSProfile(depth=10)
    node = rclpy.create_node('task')
    pub_cmd = node.create_publisher(Twist, 'cmd_vel', qos)

    # 感測器
    node.create_subscription(Sensordata,'ball_distance',  ball_distance_cb,  qos)
    node.create_subscription(Sensordata,'ball_offset',    ball_offset_cb,    qos)
    node.create_subscription(Sensordata,'ball_detected',  ball_detected_cb,  qos)
    node.create_subscription(Sensordata,'goal_distance',  goal_distance_cb,  qos)
    node.create_subscription(Sensordata,'goal_offset',    goal_offset_cb,    qos)
    node.create_subscription(Sensordata,'goal_detected',  goal_detected_cb,  qos)
    node.create_subscription(Sensordata,'trash_distance', trash_distance_cb, qos)
    node.create_subscription(Sensordata,'trash_offset',   trash_offset_cb,   qos)
    node.create_subscription(Sensordata,'trash_detected', trash_detected_cb, qos)

    node.create_subscription(Sensordata,'ball_confidence',  ball_confidence_cb,  qos)
    node.create_subscription(Sensordata,'goal_confidence',  goal_confidence_cb,  qos)
    node.create_subscription(Sensordata,'trash_confidence', trash_confidence_cb, qos)


    # 里程計
    node.create_subscription(Odometry, 'odom', odom_cb, qos)   # ★ 新增

    # ─── PID 累積項初值 ───
    pe_vis = ie_vis = pe_las = ie_las = 0.0
    count_no_ball = 0

    print('[node create !!]')

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        # ===== 追球模式 =====
        if ball_detected and ball_distance <= 75:
            uy, ie_vis = PID(Kp_visual, Ki_visual, Kd_visual,
                              ball_offset, pe_vis, ie_vis)
            pe_vis = ball_offset
            # 100 cm 內才前進；太遠就別衝
            if ball_distance <= 100:
                ux, ie_las = PID(Kp_visual, Ki_visual, Kd_laser,
                                 ball_distance, pe_las, ie_las)
                pe_las = ball_distance
            else:
                ux = 0.0
            car_control( 0.0, -uy, 0.0, pub_cmd)
            count_no_ball = 0

        # ===== 搜尋模式 =====
        else:
            count_no_ball += 1
            # （可設 > 某值才啟動，以避免相機剛掉幀就亂跑）
            dist_left  = robot_y - Y_MIN
            dist_right = Y_MAX   - robot_y
            # 先往較寬方向
            dir = -1 if dist_left > dist_right else +1
            # 快貼牆就反向
            if dist_left < MARGIN:   dir = +1
            if dist_right < MARGIN:  dir = -1
            car_control(0.0, dir * SEARCH_V, 0.0, pub_cmd)
            # car_control(0.0, 0.0, 0.0, pub_cmd)

        # （可選）列印除錯資訊
        if count_no_ball % 20 == 0:   # 每 20 迴圈印一次
            print(f'odom y={robot_y:.2f}  distL={robot_y-Y_MIN:.2f}  distR={Y_MAX-robot_y:.2f}  dir={dir if not ball_detected else 0}')
            print("\n[📊 追蹤狀態]")
            print(f"🔵 Ball  ➤ 距離: {ball_distance:.2f} cm｜偏移: {ball_offset:.2f} px｜信心: {ball_confidence:.2f}｜偵測: {'✅' if ball_detected else '❌'}")
            print(f"🟡 Goal  ➤ 距離: {goal_distance:.2f} cm｜偏移: {goal_offset:.2f} px｜信心: {goal_confidence:.2f}｜偵測: {'✅' if goal_detected else '❌'}")
            print(f"🟢 Trash ➤ 距離: {trash_distance:.2f} cm｜偏移: {trash_offset:.2f} px｜信心: {trash_confidence:.2f}｜偵測: {'✅' if trash_detected else '❌'}")
            print(f"🔧 控制參數 ➤ 前進(ux): {-ux:.2f}｜旋轉(rz): {rot:.2f}")


if __name__ == '__main__':
    main()
