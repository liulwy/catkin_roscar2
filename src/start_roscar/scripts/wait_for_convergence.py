#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动时自动对位脚本：
  1. 等待 AMCL 初始化
  2. 小车缓慢前进 ~0.3m，触发 AMCL 粒子重采样与收敛
  3. 向 /traffic_light_status 发布 "ready"，通知导航节点可以开始
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ---------- 可调参数 ----------
WIGGLE_SPEED    = 0.08      # 前移线速度 (m/s)，很慢以保证安全
WIGGLE_DURATION = 4.0       # 前移持续时间 (s)，0.08*4=0.32m > update_min_d(0.25m)
WAIT_AFTER      = 1.5       # 停车后等待 AMCL 收敛的时间 (s)
STARTUP_DELAY   = 4.0       # 等待整个系统启动的初始延时 (s)


def wait_for_convergence():
    rospy.init_node('wait_for_convergence')

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    traffic_pub = rospy.Publisher('/traffic_light_status', String, queue_size=1)

    # 1. 等待系统就绪
    rospy.loginfo("[auto_align] 等待系统启动 (%.1fs)...", STARTUP_DELAY)
    rospy.sleep(STARTUP_DELAY)

    # 2. 缓慢前移触发 AMCL 更新
    rospy.loginfo("[auto_align] 开始缓慢前移 %.2fm，触发 AMCL 收敛...",
                  WIGGLE_SPEED * WIGGLE_DURATION)

    twist = Twist()
    twist.linear.x = WIGGLE_SPEED
    twist.angular.z = 0.0

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < WIGGLE_DURATION:
        if rospy.is_shutdown():
            return
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # 3. 停车
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    rospy.loginfo("[auto_align] 前移完成，停车等待 AMCL 收敛 (%.1fs)...", WAIT_AFTER)
    rospy.sleep(WAIT_AFTER)

    # 4. 向导航节点发就绪信号
    msg = String()
    msg.data = "ready"
    traffic_pub.publish(msg)
    rospy.loginfo("[auto_align] 定位完成，已发送 ready 信号")


if __name__ == '__main__':
    try:
        wait_for_convergence()
    except rospy.ROSInterruptException:
        pass
