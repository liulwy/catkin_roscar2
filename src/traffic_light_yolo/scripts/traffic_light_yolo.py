#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import ctypes

libgomp_path = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"
if os.path.exists(libgomp_path):
    ctypes.CDLL(libgomp_path, mode=ctypes.RTLD_GLOBAL)
    os.environ.setdefault("LD_PRELOAD", libgomp_path)

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        #补充话题通信发布方
        self.cmd_pub = rospy.Publisher("/traffic_light_status", String, queue_size=10)
        self.msg = String()
        self.msg.data = "none"

        # 模型路径
        model_path = rospy.get_param("~model_path", "best.pt")

        # 加载YOLOv8模型
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 订阅图像
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # txt文件
        self.file = open("traffic_light_result.txt", "w")

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    def detect_light(self, results):

        label = "none"
        center_xy = None

        if len(results[0].boxes) == 0:
            return label, center_xy

        names = results[0].names
        boxes = results[0].boxes

        for xyxy, cls in zip(boxes.xyxy, boxes.cls):
            name = names[int(cls)]
            x1, y1, x2, y2 = xyxy.tolist()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            if name == "red":
                return "red", (cx, cy)

            if name == "green":
                label = "green"
                if center_xy is None:
                    center_xy = (cx, cy)

        return label, center_xy

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # YOLO推理
        results = self.model(frame, verbose=False,device=0)

        # 判断红绿灯
        result_label, center_xy = self.detect_light(results)

        #补充话题发布
        self.msg.data = result_label
        self.cmd_pub.publish(self.msg)

        # 写入txt
        self.file.write(result_label + "\n")

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        if center_xy is None:
            rospy.loginfo("Result: %s   XY: (none)   FPS: %.2f", result_label, fps)
        else:
            rospy.loginfo("Result: %s   XY: (%d, %d)   FPS: %.2f", result_label, center_xy[0], center_xy[1], fps)

    def shutdown(self):

        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        print("\n==============================")
        print("Total frames:", self.frame_count)
        print("Average FPS:", fps)
        print("==============================")

        self.file.close()


if __name__ == '__main__':

    detector = TrafficLightDetector()

    rospy.on_shutdown(detector.shutdown)

    rospy.spin()