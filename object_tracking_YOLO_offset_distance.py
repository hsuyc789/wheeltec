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
from ultralytics import YOLO
# Import message types and Quality of Service (QoS) profile for ROS 2
from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from custom_msg.msg import Sensordata
from geometry_msgs.msg import Twist

# Y_target
Y_target=320 #畫面中線
# Load YOLO model
model = YOLO("/home/robot/wheeltec_ros2/src/object_tracking/object_tracking/best.pt")
# Set desired confifence
CONFIDENCE_THRESHOLD = 0.6

# image processing
def image_processing(results, pub_conf, pub_distance):
    cx = Y_target
    cy = 0

    annotated_frame = results[0].orig_img.copy()
    height, width, _ = annotated_frame.shape
    center_x = width // 2
    cv2.line(annotated_frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)

    detections = results[0].boxes
    class_names = results[0].names if hasattr(results[0], 'names') else model.names
    max_conf = 0

    # 每類別對應的真實高度與焦距
    object_params = {
        0: {'real_height': 6.5, 'focal_length': 548},   # ball
        1: {'real_height': 30.0, 'focal_length': 560},  # goal 因畫面無法拍到整個目標 故無法使用
        2: {'real_height': 26.0, 'focal_length': 625}   # trash
    }

    # 初始化距離與偏移資料
    distances_by_class = {cid: -1 for cid in object_params}
    offsets_by_class = {cid: None for cid in object_params}

    for box in detections:
        conf = float(box.conf)
        if conf < CONFIDENCE_THRESHOLD:
            continue

        if conf > max_conf:
            max_conf = conf

        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        class_id = int(box.cls[0])
        label = f"{class_names[class_id]} {conf:.2f}"
        print(f"🆔 偵測到目標 ID：{class_id}，類別名稱：{class_names[class_id]}")

        # 計算像素高度、距離與偏移量
        pixel_height = abs(y2 - y1)
        real_params = object_params.get(class_id, {'real_height': 6.5, 'focal_length': 548})
        REAL_HEIGHT_CM = real_params['real_height']
        FOCAL_LENGTH_PIXELS = real_params['focal_length']
        distance_cm = (REAL_HEIGHT_CM * FOCAL_LENGTH_PIXELS) / pixel_height if pixel_height > 0 else -1
        offset_x = cx - center_x

        # 儲存該類別距離與偏移
        distances_by_class[class_id] = distance_cm
        offsets_by_class[class_id] = offset_x

        # 繪圖
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)

    # 顯示畫面
    cv2.imshow("ASR", annotated_frame)
    cv2.waitKey(1)
    pub_conf.publish(Sensordata(data=float(max_conf)))

    # 印出所有目標資訊
    print("\n📏 各目標物資訊：")
    for cid in distances_by_class:
        name = class_names[cid]
        dist = distances_by_class[cid]
        offset = offsets_by_class[cid]
        if dist > 0 and offset is not None:
            print(f"  ➤ {name}（ID {cid}）：距離 = {dist:.2f} cm，水平偏移 = {offset} px")
        else:
            print(f"  ➤ {name}（ID {cid}）：未偵測到")

    # 只發佈 ball（ID 0）距離
    if distances_by_class[0] > 0:
        distance_msg = Sensordata()
        distance_msg.data = float(distances_by_class[0])
        pub_distance.publish(distance_msg)

    # 回傳 cx 作為 Error 計算依據
    return cx + int(width / 2)



# image data callback
def image_callback(msg, pub, pub_conf, pub_distance):
    # Get the input image 
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Apply YOLO object detection model
    results = model(image, verbose=False)
    
    # image x calibration
    height, width, _ = image.shape
    # Error = int(width/2)-image_processing(results, pub_conf)+Y_target
    Error = int(width/2)-image_processing(results, pub_conf, pub_distance)+Y_target

    print(Error)
    # print(height)
    # print(width)

    if (Error<(-1*int(width/2)+25)):
        Error=0

    sensor_msg=Sensordata()
    sensor_msg.data=float(Error)
    pub.publish(sensor_msg)
        
def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('Visual_sensor')
    pub = node.create_publisher(Sensordata, 'Visual_sensor_vel', qos)
    pub_conf = node.create_publisher(Sensordata, 'Visual_sensor_confidence', qos)
    pub_distance = node.create_publisher(Sensordata, 'Visual_sensor_distance', qos) #新增

    image_sub = node.create_subscription(Image,'/image_raw',lambda msg: image_callback(msg, pub, pub_conf, pub_distance), qos)

    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
