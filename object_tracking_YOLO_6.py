#=====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
#=====================================================

#!/usr/bin/env python3
# coding=utf-8

import os
import select
import sys
import rclpy
import time
import math
import cv2
import cv_bridge
import numpy as np
import json
from ultralytics import YOLO
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from custom_msg.msg import Sensordata
from geometry_msgs.msg import Twist

Y_target = 320  # center line of screen
model = YOLO("/home/robot/wheeltec_ros2/src/object_tracking/object_tracking/best.pt")
CONFIDENCE_THRESHOLD = 0.6

def image_processing(results, pub_conf, pub_distance,
                     pub_ball_dist, pub_ball_offset,
                     pub_goal_dist, pub_goal_offset,
                     pub_trash_dist, pub_trash_offset,
                     pub_ball_detected, pub_goal_detected, pub_trash_detected,
                     pub_ball_conf, pub_goal_conf, pub_trash_conf):
    cx = Y_target
    cy = 0

    annotated_frame = results[0].orig_img.copy()
    height, width, _ = annotated_frame.shape
    center_x = width // 2
    cv2.line(annotated_frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)

    detections = results[0].boxes
    class_names = results[0].names if hasattr(results[0], 'names') else model.names
    max_conf = 0

    object_params = {
        0: {'real_height': 6.5, 'focal_length': 548},
        1: {'real_height': 30.0, 'focal_length': 560},
        2: {'real_height': 26.0, 'focal_length': 625}
    }

    distances_by_class = {cid: -1 for cid in object_params}
    offsets_by_class = {cid: None for cid in object_params}
        # 預設為未偵測到
    detected_flags = {0: 0.0, 1: 0.0, 2: 0.0}
    confidence_values = {0: 0.0, 1: 0.0, 2: 0.0}


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

        detected_flags[class_id] = 1.0
        confidence_values[class_id] = conf


        pixel_height = abs(y2 - y1)
        real_params = object_params.get(class_id, {'real_height': 6.5, 'focal_length': 548})
        REAL_HEIGHT_CM = real_params['real_height']
        FOCAL_LENGTH_PIXELS = real_params['focal_length']
        distance_cm = (REAL_HEIGHT_CM * FOCAL_LENGTH_PIXELS) / pixel_height if pixel_height > 0 else -1
        offset_x = cx - center_x

        distances_by_class[class_id] = distance_cm
        offsets_by_class[class_id] = offset_x

        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)

    cv2.imshow("ASR", annotated_frame)
    cv2.waitKey(1)
    pub_conf.publish(Sensordata(data=float(max_conf)))
        # 發佈各個物體的是否被偵測到與 confidence
    pub_ball_detected.publish(Sensordata(data=detected_flags[0]))
    pub_goal_detected.publish(Sensordata(data=detected_flags[1]))
    pub_trash_detected.publish(Sensordata(data=detected_flags[2]))

    pub_ball_conf.publish(Sensordata(data=confidence_values[0]))
    pub_goal_conf.publish(Sensordata(data=confidence_values[1]))
    pub_trash_conf.publish(Sensordata(data=confidence_values[2]))


    print("\n📏 各目標物距離與偏移：")
    for cid in distances_by_class:
        name = class_names[cid]
        dist = distances_by_class[cid]
        offset = offsets_by_class[cid]
        if dist > 0 and offset is not None:
            print(f"  ➔ {name}（ID {cid}）：距離 = {dist:.2f} cm，偏移 = {offset} px")
        else:
            print(f"  ➔ {name}（ID {cid}）：未偵測到")

    if distances_by_class[0] > 0:
        pub_distance.publish(Sensordata(data=float(distances_by_class[0])))
    if distances_by_class[0] > 0 and offsets_by_class[0] is not None:
        pub_ball_dist.publish(Sensordata(data=float(distances_by_class[0])))
        pub_ball_offset.publish(Sensordata(data=float(offsets_by_class[0])))
    if distances_by_class[1] > 0 and offsets_by_class[1] is not None:
        pub_goal_dist.publish(Sensordata(data=float(distances_by_class[1])))
        pub_goal_offset.publish(Sensordata(data=float(offsets_by_class[1])))
    if distances_by_class[2] > 0 and offsets_by_class[2] is not None:
        pub_trash_dist.publish(Sensordata(data=float(distances_by_class[2])))
        pub_trash_offset.publish(Sensordata(data=float(offsets_by_class[2])))

    return cx + int(width / 2)

def image_callback(msg, pub, pub_conf, pub_distance,
                   pub_ball_dist, pub_ball_offset,
                   pub_goal_dist, pub_goal_offset,
                   pub_trash_dist, pub_trash_offset):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    results = model(image, verbose=False)
    height, width, _ = image.shape
    Error = int(width/2) - image_processing(
        results, pub_conf, pub_distance,
        pub_ball_dist, pub_ball_offset,
        pub_goal_dist, pub_goal_offset,
        pub_trash_dist, pub_trash_offset,
        pub_ball_detected, pub_goal_detected, pub_trash_detected,
        pub_ball_conf, pub_goal_conf, pub_trash_conf
    ) + Y_target


    print(Error)
    if Error < (-1 * int(width / 2) + 25):
        Error = 0

    sensor_msg = Sensordata()
    sensor_msg.data = float(Error)
    pub.publish(sensor_msg)

def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('Visual_sensor')
    pub = node.create_publisher(Sensordata, 'Visual_sensor_vel', qos)
    pub_conf = node.create_publisher(Sensordata, 'Visual_sensor_confidence', qos)
    pub_distance = node.create_publisher(Sensordata, 'Visual_sensor_distance', qos)

    pub_ball_dist = node.create_publisher(Sensordata, 'ball_distance', qos)
    pub_ball_offset = node.create_publisher(Sensordata, 'ball_offset', qos)
    pub_goal_dist = node.create_publisher(Sensordata, 'goal_distance', qos)
    pub_goal_offset = node.create_publisher(Sensordata, 'goal_offset', qos)
    pub_trash_dist = node.create_publisher(Sensordata, 'trash_distance', qos)
    pub_trash_offset = node.create_publisher(Sensordata, 'trash_offset', qos)

    pub_ball_detected = node.create_publisher(Sensordata, 'ball_detected', qos)
    pub_goal_detected = node.create_publisher(Sensordata, 'goal_detected', qos)
    pub_trash_detected = node.create_publisher(Sensordata, 'trash_detected', qos)

    pub_ball_conf = node.create_publisher(Sensordata, 'ball_confidence', qos)
    pub_goal_conf = node.create_publisher(Sensordata, 'goal_confidence', qos)
    pub_trash_conf = node.create_publisher(Sensordata, 'trash_confidence', qos)


    image_sub = node.create_subscription(
        Image, '/image_raw',
        lambda msg: image_callback(
            msg, pub, pub_conf, pub_distance,
            pub_ball_dist, pub_ball_offset,
            pub_goal_dist, pub_goal_offset,
            pub_trash_dist, pub_trash_offset
        ),
        qos
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
