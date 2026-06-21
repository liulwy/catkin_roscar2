#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘遥控 + 路径录制脚本
  开车:  w=前进  x=后退  a=左转  d=右转  s=停止
  录制:  r=开始/暂停录制  p=保存路径  c=清除/放弃录制
  调速:  1/2=减/增线速度  3/4=减/增角速度
  退出:  q
"""

import os
import sys
import select
import termios
import tty
import threading
import rospy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

HELP_MSG = """
----------------------------------------------------
  键盘遥控 + 路径录制
----------------------------------------------------
  开车:  w=前进  x=后退  a=左转  d=右转  s=停止
  录制:  r=开始/暂停录制  p=保存路径到YAML  c=清除录制
  调速:  1/2=减/增线速度  3/4=减/增角速度
  退出:  q
----------------------------------------------------
"""

# ===== 路径录制状态 =====
recording = False
recorded_waypoints = []
last_sample_pose = None       # (x, y, yaw)
sample_distance = 0.3         # 每隔0.3m采样一个点
odom_lock = threading.Lock()

def odom_callback(msg):
    """后台订阅 /odom_combined，录制时自动按距离采样"""
    global recording, recorded_waypoints, last_sample_pose

    if not recording:
        return

    with odom_lock:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if last_sample_pose is not None:
            dx = x - last_sample_pose[0]
            dy = y - last_sample_pose[1]
            if (dx * dx + dy * dy) < (sample_distance * sample_distance):
                return  # 还不够远，跳过

        recorded_waypoints.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
        last_sample_pose = (x, y, yaw)
        rospy.loginfo("  [录制] 采样点 #%d: (%.3f, %.3f, %.3f)",
                      len(recorded_waypoints), x, y, yaw)


def get_key(timeout=0.1):
    """非阻塞读取单键（需在 setraw 之后调用）"""
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

    # 确保保存目录存在
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # ----- 发布 & 订阅 -----
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    odom_sub = rospy.Subscriber("/odom_combined", Odometry, odom_callback, queue_size=1)

    twist = Twist()

    rospy.loginfo(HELP_MSG)
    rospy.loginfo("linear=%.2f  angular=%.2f  录制采样间距=%.1fm  保存路径=%s",
                  linear, angular, sample_distance, save_dir)

    # ----- 终端 raw 模式（一次性设置）-----
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)

        while not rospy.is_shutdown():
            key = get_key(0.1)

            # --- 驱动键 ---
            if key == "w":
                twist.linear.x = linear
                twist.angular.z = 0.0
            elif key == "x":
                twist.linear.x = -linear
                twist.angular.z = 0.0
            elif key == "a":
                twist.linear.x = 0.0
                twist.angular.z = angular
            elif key == "d":
                twist.linear.x = 0.0
                twist.angular.z = -angular
            elif key == "s":
                twist.linear.x = 0.0
                twist.angular.z = 0.0

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
                        last_sample_pose = None  # 重置参考点，确保录制开始即刻采样
                        rospy.loginfo(">>> 开始录制路径（采样间距=%.1fm）", sample_distance)
                    else:
                        rospy.loginfo(">>> 暂停录制（已录制 %d 个点）", len(recorded_waypoints))
                continue

            elif key == "p":
                with odom_lock:
                    if not recorded_waypoints:
                        rospy.logwarn("没有录制点，请先按 r 开始录制")
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

            else:
                # 未识别的键：不发送速度（避免无指令导致的意外）
                continue

            pub.publish(twist)

    finally:
        # 恢复终端设置 + 停车
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rospy.loginfo("键盘遥控已停止，机器人已停车")
