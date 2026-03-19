#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import ctypes

libgomp_path = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"
if os.path.exists(libgomp_path):
    ctypes.CDLL(libgomp_path, mode=ctypes.RTLD_GLOBAL)
    os.environ.setdefault("LD_PRELOAD", libgomp_path)
os.environ["LD_PRELOAD"]="/usr/lib/aarch64-linux-gnu/libgomp.so.1"


import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String

OPEN_ANNOTED = 0

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        #补充话题通信发布方
        self.cmd_pub = rospy.Publisher("/traffic_light_status", String, queue_size=10)

        if OPEN_ANNOTED == 1:
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
        
        # [新增] 打开存放雷达数据的文件 (路径为 workspace 根目录)
        self.lidar_file = open("/home/gdut/catkin_roscar2/lidar_ranges.txt", "w")

        # 雷达测距相关
        # 优先从yaml读取相机内参
        self.camera_cx = None
        self.camera_fx = None
        try:
            import yaml
            yaml_path = rospy.get_param('~cam_lidar_matrix_path', "/home/gdut/catkin_roscar2/src/start_roscar/param/cam_lidar_matrix.yaml")
            with open(yaml_path, 'r') as f:
                cam_yaml = yaml.safe_load(f)
            cam_matrix = cam_yaml['camera']['camera_matrix']['data']
            self.camera_fx = cam_matrix[0]
            self.camera_cx = cam_matrix[2]
            rospy.loginfo("[cam_lidar_matrix.yaml] 加载相机内参 fx=%.3f, cx=%.3f", self.camera_fx, self.camera_cx)
        except Exception as e:
            rospy.logwarn("未能从 cam_lidar_matrix.yaml 读取相机内参: %s，使用参数服务器默认值", e)
            self.camera_cx = rospy.get_param('~camera_cx', 320.0)
            self.camera_fx = rospy.get_param('~camera_fx', 500.0)

        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.lidar_angle_max = 0.0
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # 雷达相对相机的安装偏移角（弧度）
        # 根据实测数据：相机角度-4.91°对应雷达177.9°，故偏移=182.81°=3.1906rad
        # 可通过ROS参数 ~lidar_offset_angle 覆盖
        self.lidar_offset_angle = rospy.get_param('~lidar_offset_angle', 3.1906)

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    def lidar_callback(self, msg):
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_angle_max = msg.angle_max
        self.lidar_ranges = list(msg.ranges)

        # [修改] 将雷达360°数据(角度, 距离)写入文件
        # 计算每个点对应的角度（度）
        if hasattr(self, 'lidar_file'):
            data_with_angles = []
            for i, distance in enumerate(self.lidar_ranges):
                # 计算当前索引对应的弧度
                angle_rad = self.lidar_angle_min + i * self.lidar_angle_increment
                # 转换为角度
                angle_deg = np.degrees(angle_rad)
                # 保留3位小数
                data_with_angles.append((round(angle_deg, 2), round(distance, 3)))
            
            self.lidar_file.write(str(data_with_angles) + "\n")

        # 调试：打印雷达基本参数（只打印一次）
        if not hasattr(self, '_lidar_debug_printed'):
            self._lidar_debug_printed = True
            total = len(self.lidar_ranges)
            idx_front = int(round((0.0 - msg.angle_min) / msg.angle_increment)) % total
            rospy.logwarn("=== 雷达参数 ===")
            rospy.logwarn("angle_min=%.4f rad (%.1f°)", msg.angle_min, np.degrees(msg.angle_min))
            rospy.logwarn("angle_max=%.4f rad (%.1f°)", msg.angle_max, np.degrees(msg.angle_max))
            rospy.logwarn("angle_increment=%.6f rad (%.4f°)", msg.angle_increment, np.degrees(msg.angle_increment))
            rospy.logwarn("总点数: %d, 正前方索引: %d", total, idx_front)
            rospy.logwarn("正前方附近距离(idx %d±3): %s", idx_front,
                [round(self.lidar_ranges[(idx_front+i) % total], 3) for i in range(-3, 4)])

    def calculate_distance(self, u, label=""):
        if not self.lidar_ranges or self.lidar_angle_increment == 0:
            return float('inf')

        # 1. 计算像素点对应的相机水平视场角（相对相机正前方）
        theta_rad = np.arctan2(self.camera_cx - u, self.camera_fx)  # 弧度，范围 -π/2 到 π/2

        # 2. 加上雷达安装偏移角，转换到雷达坐标系下的绝对角度
        theta_lidar = theta_rad + self.lidar_offset_angle

        # 3. 将角度归一化到 [angle_min, angle_min + 2π)
        angle_diff = theta_lidar - self.lidar_angle_min
        # 归一化到 [0, 2π)
        angle_diff = angle_diff % (2 * np.pi)

        idx_center = int(round(angle_diff / self.lidar_angle_increment))
        idx_center = idx_center % len(self.lidar_ranges)

        # 3. 查找 +-2° 的雷达数据
        angle_range = np.radians(2)
        idx_offset = int(round(angle_range / self.lidar_angle_increment))

        valid_distances = []
        raw_data = []

        # 完整提取 +-2° 内的所有原始数据用于打印
        for i in range(-idx_offset, idx_offset + 1):
            idx = (idx_center + i) % len(self.lidar_ranges)
            d = self.lidar_ranges[idx]
            raw_data.append(round(d, 3))
            if not np.isinf(d) and not np.isnan(d) and d > 0.05:
                valid_distances.append(d)

        # 取有效距离的最小值（更适合红绿灯距离判定）
        if valid_distances:
            distance = np.min(valid_distances)
        else:
            distance = float('inf')

        rospy.loginfo("检测到 %s 灯, 目标像素: u=%d, 相机角度: %.2f°, 雷达角度: %.2f°, 雷达距离(最小): %.3fm",
                      label, u, np.degrees(theta_rad), np.degrees(theta_lidar), distance)
        # rospy.loginfo("目标角度+-2°内原始雷达数据(米): %s", raw_data)

        # 调试：打印角度映射中间值
        # total = len(self.lidar_ranges)
        # idx_front = int(round((self.lidar_offset_angle - self.lidar_angle_min) / self.lidar_angle_increment)) % total
        # rospy.logwarn("=== 角度映射调试 ===")
        # rospy.logwarn("camera_cx=%.1f, camera_fx=%.1f, u=%d", self.camera_cx, self.camera_fx, u)
        # rospy.logwarn("theta_rad=%.4f (%.2f°), theta_lidar=%.4f (%.2f°)", theta_rad, np.degrees(theta_rad), theta_lidar, np.degrees(theta_lidar))
        # rospy.logwarn("idx_center=%d, idx_front=%d, 差值=%d", idx_center, idx_front, idx_center - idx_front)
        # rospy.logwarn("相机正前方(idx=%d)距离: %.3fm, 目标(idx=%d)距离: %.3fm", idx_front, self.lidar_ranges[idx_front % total], idx_center, self.lidar_ranges[idx_center % total])

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

        if OPEN_ANNOTED == 1:
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


        # [新增] 关闭雷达数据文件
        if hasattr(self, 'lidar_file'):
            self.lidar_file.close()


if __name__ == '__main__':

    detector = TrafficLightDetector()

    rospy.on_shutdown(detector.shutdown)

    rospy.spin()