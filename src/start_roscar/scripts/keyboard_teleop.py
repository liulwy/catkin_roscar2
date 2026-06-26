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

    try:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    except Exception as e:
        rospy.logerr("[odom_callback] 解析里程计数据异常: %s", e)
        return

    with pose_lock:
        latest_pose = (x, y, yaw)

    # 诊断日志：每秒打印一次，确认回调是否被触发
    rospy.loginfo_throttle(1.0, "[odom_callback] 收到里程计: x=%.3f y=%.3f yaw=%.3f", x, y, yaw)

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
        _print("[录制] #%d  (%.3f, %.3f, %.3f)" % (len(recorded_waypoints), x, y, yaw))


def get_key(timeout=0.05):
    """非阻塞读取单键（需在 setraw 之后调用），超时返回空字符串"""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


def _print(msg):
    """raw 模式下安全打印：\r\n 确保回车+换行，flush 确保立即输出"""
    sys.stdout.write(msg + "\r\n")
    sys.stdout.flush()


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

    # 诊断：等待 /odom 话题首次连接，超时 5 秒
    rospy.loginfo("等待 /odom 话题连接...")
    start_wait = rospy.Time.now()
    while odom_sub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - start_wait).to_sec() > 5.0:
            rospy.logwarn("/odom 话题 5 秒内无发布者连接！请检查 driver_node 是否启动。")
            break
        rospy.sleep(0.1)
    if odom_sub.get_num_connections() > 0:
        rospy.loginfo("/odom 话题已连接，发布者数量: %d", odom_sub.get_num_connections())

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
                _print("线速度: %.2f m/s" % linear)
                continue
            elif key == "2":
                linear = linear + linear_step
                _print("线速度: %.2f m/s" % linear)
                continue
            elif key == "3":
                angular = max(0.0, angular - angular_step)
                _print("角速度: %.2f rad/s" % angular)
                continue
            elif key == "4":
                angular = angular + angular_step
                _print("角速度: %.2f rad/s" % angular)
                continue
            # --- 录制键 ---
            elif key == "r":
                with odom_lock:
                    recording = not recording
                    if recording:
                        last_sample_pose = None
                        _print("\n>>> 开始录制路径（采样间距=%.1fm）" % sample_distance)
                    else:
                        _print(">>> 暂停录制（已录制 %d 个点）\n" % len(recorded_waypoints))
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
                        _print("[警告] 没有录制点，请先按 r 或 j 添加点位")
                    else:
                        # 按时间戳命名，不覆盖旧文件
                        filename = "patrol_path_%s.yaml" % time.strftime("%m_%d_%H_%M")
                        filepath = os.path.join(save_dir, filename)
                        with open(filepath, 'w') as f:
                            f.write("# 路径名称\n")
                            f.write('path_name: "recorded_path"\n')
                            f.write("\n")
                            f.write("# 路径点列表 — 由键盘遥控录制\n")
                            f.write("waypoints:\n")
                            for i, wp in enumerate(recorded_waypoints):
                                f.write("  # 点位 %d\n" % (i + 1))
                                f.write("  - {x: %.4f, y: %.4f, yaw: %.4f}\n" %
                                        (wp['x'], wp['y'], wp['yaw']))
                        _print(">>> 路径已保存: %s（共 %d 个点）" % (filepath, len(recorded_waypoints)))
                        recording = False
                continue
            elif key == "c":
                with odom_lock:
                    recording = False
                    recorded_waypoints = []
                    last_sample_pose = None
                    _print(">>> 录制已清除")
                continue
            # --- 退出键 ---
            elif key == "q":
                _print("退出键盘遥控")
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
            # 出让 GIL 给 subscriber 回调线程，防止 CPU 高负载时回调被饿死
            rospy.sleep(0.001)

    finally:
        # 恢复终端 + 停车
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rospy.loginfo("键盘遥控已停止，机器人已停车")
