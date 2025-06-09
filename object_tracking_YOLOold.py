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
Y_target=320
# Load YOLO model
model = YOLO("/home/robot/wheeltec_ros2/src/object_tracking/object_tracking/best.pt")
# Set desired confifence
CONFIDENCE_THRESHOLD = 0.85

# image processing
def image_processing(results):
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

    for box in detections:
        conf = float(box.conf)
        if conf < CONFIDENCE_THRESHOLD:
            continue  # Skip low-confidence

        # Coordinates
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Class label
        class_id = int(box.cls[0])
        label = f"{class_names[class_id]} {conf:.2f}"

        # Draw bounding box
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Draw label
        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Draw centroid
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)

    # Show the image
    cv2.imshow("ASR", annotated_frame)
    cv2.waitKey(1)

    return cx + int(width / 2)

# image data callback
def image_callback(msg, pub):
    # Get the input image 
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Apply YOLO object detection model
    results = model(image, verbose=False)
    
    # image x calibration
    height, width, _ = image.shape
    Error = int(width/2)-image_processing(results)+Y_target
    print(Error)

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
    image_sub = node.create_subscription(Image,'/image_raw',lambda msg: image_callback(msg, pub), qos)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
