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
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        #补充话题通信发布方
        self.cmd_pub = rospy.Publisher("/traffic_light_status", String, queue_size=10)
        self.image_pub = rospy.Publisher("/traffic_light_yolo/image_annotated", Image, queue_size=1)
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

        # 雷达测距相关
        self.camera_cx = rospy.get_param('~camera_cx', 320.0)
        self.camera_fx = rospy.get_param('~camera_fx', 500.0)
        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.lidar_angle_max = 0.0
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    def lidar_callback(self, msg):
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_angle_max = msg.angle_max
        self.lidar_ranges = list(msg.ranges)

    def calculate_distance(self, u, label=""):
        if not self.lidar_ranges or self.lidar_angle_increment == 0:
            return float('inf')

        # 1. 计算像素点对应的雷达水平视场角 (坐标系对齐)
        # 相机图像坐标系 u 轴向右递增；而 ROS 标准雷达坐标系通常逆时针递增（正前方为0，向左为正角度）。
        # 当 u < cx (物体在画面左侧) 时，(cx - u) 为正，对应雷达正角度（向左偏）。
        theta = np.arctan2(self.camera_cx - u, self.camera_fx)  # 单位：弧度
            
        # 2. 查找 +-5° 的雷达数据
        angle_range = np.radians(5)
        # 使用 % (2*np.pi) 可以兼容所有类型的 lidar_angle_min (如 -pi~pi 或 0~2pi) 并自动支持越界处理
        angle_diff = (theta - self.lidar_angle_min) % (2 * np.pi)
        idx_center = int(round(angle_diff / self.lidar_angle_increment))
        idx_center = idx_center % len(self.lidar_ranges)
        idx_offset = int(round(angle_range / self.lidar_angle_increment))
        
        valid_distances = []
        raw_data = []
        
        # 完整提取 +-5° 内包括越界回环的所有原始数据用于打印
        for i in range(-idx_offset, idx_offset + 1):
            idx = (idx_center + i) % len(self.lidar_ranges)
            d = self.lidar_ranges[idx]
            raw_data.append(round(d, 3))
            if not np.isinf(d) and not np.isnan(d) and d > 0.05:
                valid_distances.append(d)
                
        # 以前我们通过取“最小值”容易误抓取镜头边缘的较近障碍物（如1.06m）。
        # 此处改为：从正中心角度往左右两边排查，返回距离红绿灯光学角度最接近的有效雷达距离！
        distance = float('inf')
        for i in range(idx_offset + 1):
            # 看右侧偏差(0, 1, 2...)
            idx_r = (idx_center + i) % len(self.lidar_ranges)
            d_r = self.lidar_ranges[idx_r]
            if not np.isinf(d_r) and not np.isnan(d_r) and d_r > 0.05:
                distance = d_r
                break
                
            # 看左侧偏差(-1, -2...)
            if i != 0:
                idx_l = (idx_center - i + len(self.lidar_ranges)) % len(self.lidar_ranges)
                d_l = self.lidar_ranges[idx_l]
                if not np.isinf(d_l) and not np.isnan(d_l) and d_l > 0.05:
                    distance = d_l
                    break
                
        rospy.loginfo("检测到 %s 灯, 目标像素: u=%d, 计算角度: %.2f°, 雷达距离: %.3fm", label, u, np.degrees(theta), distance)
        rospy.loginfo("目标角度+-5°内原始雷达数据(米): %s", raw_data)
        return distance

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
        # 如果所在设备是 Jetson Nano (Maxwell架构)，它对 FP16 (half) 支持不好，容易报找不到Engine或显存溢出。
        # 这里关闭 half=True，仅使用 imgsz=320 以降低显存占用。如果仍旧 OOM，请将 device=0 改为 device='cpu'。
        results = self.model(frame, verbose=False, device=0, imgsz=320)

        # 判断红绿灯
        result_label, center_xy = self.detect_light(results)

        # 把颜色和距离拼接成字符串，例如 "red,1.25"
        if center_xy is not None:
            distance = self.calculate_distance(center_xy[0], result_label)
            if np.isinf(distance):
                distance = -1.0
            formatted_msg = f"{result_label},{distance:.3f}"
        else:
            formatted_msg = f"{result_label},-1.0"

        # 发布拼接后的话题
        self.msg.data = formatted_msg
        self.cmd_pub.publish(self.msg)

        # 发布 YOLO 识别可视化图像（带检测框）
        annotated_frame = results[0].plot()
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        image_msg.header = msg.header
        self.image_pub.publish(image_msg)

        # 写入txt
        self.file.write(result_label + "\n")

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        #rospy.loginfo("Result: %s   FPS: %.2f", result_label, fps)


        
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