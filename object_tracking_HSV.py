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

# Y_target
Y_target=0

# image processing
def image_processing(image):
    # Read HSV value
    file_path = os.path.join(os.getcwd(), 'src/object_tracking/object_tracking/hsv_values.json')
    with open(file_path, 'r') as f:
    	hsv_values = json.load(f)
    # HSV color recognition
    kernel = numpy.ones((5,5),numpy.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_erode = cv2.erode(hsv,kernel,iterations=1)
    hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
    mask=cv2.inRange(hsv_dilate,(hsv_values[0],hsv_values[1],hsv_values[2]),(hsv_values[3],hsv_values[4],hsv_values[5]))
    masked_img = cv2.bitwise_and(image, image, mask=mask)
    # Morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    img_erode = cv2.erode(mask, kernel, iterations=1)
    img_dilated = cv2.dilate(mask, kernel, iterations=1)
    # findContours
    height, width, _ = image.shape
    contours, hierarchy = cv2.findContours(img_dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x=0
    w=0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h
        if area<3000:
            continue
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.line(image, (x + int(w/2), y), (x + int(w/2), y+h), (185, 218, 255), 2)
        cv2.line(image, (int(width/2), 0), (int(width/2), height), (255, 255, 187), 2)
    # Plot the image
    cv2.imshow("ASR", image)
    cv2.waitKey(1)
    
    
    return x + int(w/2)
    

# image data callback
def image_callback(msg, pub):
    # Get the input image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # image y calibration
    height, width, _ = image.shape
    Error = int(width/2)-image_processing(image)+Y_target

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
