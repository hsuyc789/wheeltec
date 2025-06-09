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
CONFIDENCE_THRESHOLD = 0.6

# image processing
def image_processing(results, pub_conf, pub_ball_position_x, pub_ball_width):
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

    # Ball position
    confidence_ball_width = -1 # Store the width of the ball that is the most confident.
    confidence_ball_position_x = -1 # Store the position of the ball that is the most confident.
    max_ball_confidence = 0.0

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
        class_name = class_names[class_id]
        label = f"{class_name} {conf:.2f}"

        if class_name == "ball":
            # Update ball position
            if conf > max_ball_confidence:
                max_ball_confidence = conf
                confidence_ball_width = int(abs(x2 - x1))
                confidence_ball_position_x = cx

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

    pub_conf.publish(Sensordata(data=float(max_conf)))
    pub_ball_position_x.publish(Sensordata(data=float(confidence_ball_position_x)))
    pub_ball_width.publish(Sensordata(data=float(confidence_ball_width)))

    return cx + int(width / 2)

# image data callback
def image_callback(msg, pub, pub_conf, pub_ball_position_x, pub_ball_width):
    # Get the input image 
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Apply YOLO object detection model
    results = model(image, verbose=False)
    
    # image x calibration
    height, width, _ = image.shape
    Error = int(width/2)-image_processing(results, pub_conf, pub_ball_position_x, pub_ball_width)+Y_target
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
    pub_conf = node.create_publisher(Sensordata, 'Visual_sensor_confidence', qos)
    pub_ball_position_x = node.create_publisher(Sensordata, 'Visual_ball_position_x', qos)
    pub_ball_width = node.create_publisher(Sensordata, 'Visual_ball_width', qos)
    image_sub = node.create_subscription(Image,'/image_raw',lambda msg: image_callback(msg, pub, pub_conf, pub_ball_position_x, pub_ball_width), qos)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
