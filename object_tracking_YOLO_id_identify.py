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
def image_processing(results, pub_conf, pub_distance): #改過
    # Initialize the variable
    cx = Y_target
    cy = 0

    # Start from original frame
    annotated_frame = results[0].orig_img.copy()

    # Frame dimensions
    height, width, _ = annotated_frame.shape
    center_x = width // 2

    # Draw vertical center line
    cv2.line(annotated_frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)

    # Get boxes, confidences, and class indices
    detections = results[0].boxes
    class_names = results[0].names if hasattr(results[0], 'names') else model.names

    max_conf = 0

    for box in detections:
        conf = float(box.conf)
        if conf < CONFIDENCE_THRESHOLD:
            continue  # Skip low-confidence
        
        if conf > max_conf:
            max_conf = conf
        # Coordinates
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Class label
        class_id = int(box.cls[0])
        label = f"{class_names[class_id]} {conf:.2f}"

        print(f"🆔 偵測到目標 ID：{class_id}，類別名稱：{class_names[class_id]}") #新增 id0 ball, id1 goal, id2 trash


        # Draw bounding box
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Draw label
        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Draw centroid
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)

        # # 目標物大小測試 新增 
        pixel_width  = abs(x2 - x1)
        pixel_height = abs(y2 - y1)
        print(f"🎯 目標框大小：寬度 = {pixel_width} px，高度 = {pixel_height} px")

        # 目標在畫面中的像素高度 新增
        pixel_height = abs(y2 - y1)

        # ==== 設定已知參數 ====
        REAL_HEIGHT_CM = 6.5  # 例如：球6.5cm 在距離50cm下 像素高度80 焦距校正後548 trash 28cm 距離73cm pixel230 焦距600
        FOCAL_LENGTH_PIXELS = 615  # 像素高*距離/真實高度＝焦距
        distance_cm = 0
        if pixel_height > 0:
            distance_cm = (REAL_HEIGHT_CM * FOCAL_LENGTH_PIXELS) / pixel_height   
        else:
            distance_cm = -1  # 無法計算距離（避免除以0）
        print(f"🎯 目標物距離：距離 = {distance_cm}")

        distance_msg = Sensordata() #新增
        distance_msg.data = float(distance_cm) #新增
        pub_distance.publish(distance_msg) #新增

    
    # Show the image
    cv2.imshow("ASR", annotated_frame)
    cv2.waitKey(1)

    pub_conf.publish(Sensordata(data=float(max_conf)))

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
