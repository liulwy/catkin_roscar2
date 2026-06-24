#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘遥控 + 路径录制脚本（小乌龟模式：按住动，松开停，支持组合键）
  开车:  w=前进  s=后退  a=左转  d=右转
         w+d=前进右转  w+a=前进左转  s+d=后退右转  s+a=后退左转
  录制:  r=自动录制  j=手动打点  p=保存路径  c=清除
  调速:  1/2=减/增线速度  3/4=减/增角速度
  退出:  q
"""

import os
import sys
import select
import termios
import tty
import time
import threading
import rospy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

HELP_TEXT = """\
----------------------------------------------------
  键盘遥控 + 路径录制（小乌龟模式）
----------------------------------------------------
  开车:  w=前进  s=后退  a=左转  d=右转
         w+d=前进右转  w+a=前进左转
         s+d=后退右转  s+a=后退左转
  录制:  r=自动录制  j=手动打点  p=保存路径  c=清除
  调速:  1/2=减/增线速度  3/4=减/增角速度
  退出:  q
----------------------------------------------------"""

# ===== 路径录制状态 =====
recording = False
recorded_waypoints = []
last_sample_pose = None       # (x, y, yaw) for auto-recording distance check
sample_distance = 0.3         # 自动录制：每隔0.3m采样一个点
odom_lock = threading.Lock()

# ===== 最新里程计位姿（手动打点用）=====
latest_pose = None            # (x, y, yaw)
pose_lock = threading.Lock()

def odom_callback(msg):
    """后台订阅 /odom，始终缓存最新位姿；自动录制时按距离采样"""
    global recording, recorded_waypoints, last_sample_pose, latest_pose

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    with pose_lock:
        latest_pose = (x, y, yaw)

    if not recording:
        return

    with odom_lock:
        if last_sample_pose is not None:
            dx = x - last_sample_pose[0]
            dy = y - last_sample_pose[1]
            if (dx * dx + dy * dy) < (sample_distance * sample_distance):
                return

        recorded_waypoints.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
        last_sample_pose = (x, y, yaw)
        rospy.loginfo("[录制] 采样点 #%d: (%.3f, %.3f, %.3f)",
                      len(recorded_waypoints), x, y, yaw)


def get_key(timeout=0.05):
    """非阻塞读取单键（需在 setraw 之后调用），超时返回空字符串"""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


if __name__ == "__main__":
    rospy.init_node("keyboard_teleop_node")

    # ----- 参数 -----
    linear       = rospy.get_param("~linear", 0.15)
    angular      = rospy.get_param("~angular", 0.8)
    linear_step  = rospy.get_param("~linear_step", 0.02)
    angular_step = rospy.get_param("~angular_step", 0.05)
    save_dir     = rospy.get_param("~save_dir",
                                   os.path.join(os.path.expanduser("~"),
                                                "catkin_roscar2/src/start_roscar/path"))

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # ----- 发布 & 订阅 -----
    pub    = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    twist = Twist()

    print(HELP_TEXT)
    print("线性速度: %.2f m/s | 角速度: %.2f rad/s | 录制间距: %.1fm | 保存路径: %s" %
          (linear, angular, sample_distance, save_dir))

    # ----- 终端 raw 模式 -----
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # ===== WASD 按键状态（小乌龟模式：按住动，松开停）=====
    key_state = {'w': False, 's': False, 'a': False, 'd': False}
    last_drive_key_time = 0.0   # 最后一次收到驱动键的时间戳
    HOLD_TIMEOUT = 0.12         # 超过这个时间没收到驱动键就认为松开了

    try:
        tty.setraw(fd)

        while not rospy.is_shutdown():
            key = get_key(0.05)   # 50ms 轮询，兼顾响应速度和 CPU
            now = time.time()

            # --- 驱动键: 记录按下状态 ---
            if key in key_state:
                key_state[key] = True
                last_drive_key_time = now
            # --- 调速键 ---
            elif key == "1":
                linear = max(0.0, linear - linear_step)
                rospy.loginfo("线速度: %.2f m/s", linear)
                continue
            elif key == "2":
                linear = linear + linear_step
                rospy.loginfo("线速度: %.2f m/s", linear)
                continue
            elif key == "3":
                angular = max(0.0, angular - angular_step)
                rospy.loginfo("角速度: %.2f rad/s", angular)
                continue
            elif key == "4":
                angular = angular + angular_step
                rospy.loginfo("角速度: %.2f rad/s", angular)
                continue
            # --- 录制键 ---
            elif key == "r":
                with odom_lock:
                    recording = not recording
                    if recording:
                        last_sample_pose = None
                        rospy.loginfo(">>> 开始录制路径（采样间距=%.1fm）", sample_distance)
                    else:
                        rospy.loginfo(">>> 暂停录制（已录制 %d 个点）", len(recorded_waypoints))
                continue
            elif key == "j":
                with pose_lock, odom_lock:
                    if latest_pose is None:
                        rospy.logwarn("尚未收到里程计数据，无法打点")
                    else:
                        x, y, yaw = latest_pose
                        recorded_waypoints.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
                        idx = len(recorded_waypoints)
                        rospy.loginfo(
                            "\n=========================================\n"
                            "  [手动打点] 路径点 #%d:\n"
                            "    x=%.3f  y=%.3f  yaw=%.3f\n"
                            "=========================================\n",
                            idx, x, y, yaw)
                continue
            elif key == "p":
                with odom_lock:
                    if not recorded_waypoints:
                        rospy.logwarn("没有录制点，请先按 r 或 j 添加点位")
                    else:
                        filename = "patrol_path.yaml"
                        filepath = os.path.join(save_dir, filename)
                        data = {
                            'path_name': 'recorded_path',
                            'waypoints': recorded_waypoints,
                        }
                        with open(filepath, 'w') as f:
                            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
                        rospy.loginfo(">>> 路径已保存: %s（共 %d 个点）",
                                      filepath, len(recorded_waypoints))
                        recording = False
                continue
            elif key == "c":
                with odom_lock:
                    recording = False
                    recorded_waypoints = []
                    last_sample_pose = None
                    rospy.loginfo(">>> 录制已清除")
                continue
            # --- 退出键 ---
            elif key == "q":
                rospy.loginfo("退出键盘遥控")
                break

            # --- 松键检测：超过 HOLD_TIMEOUT 没收到驱动键，全部清零 ---
            if now - last_drive_key_time > HOLD_TIMEOUT:
                for k in key_state:
                    key_state[k] = False

            # --- 计算 Twist（组合键自然支持）---
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key_state['w']:
                twist.linear.x += linear
            if key_state['s']:
                twist.linear.x -= linear
            if key_state['a']:
                twist.angular.z += angular
            if key_state['d']:
                twist.angular.z -= angular

            pub.publish(twist)

    finally:
        # 恢复终端 + 停车
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rospy.loginfo("键盘遥控已停止，机器人已停车")
