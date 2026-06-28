#!/usr/bin/env python3
"""
静态 TF 发布器 —— 发布传感器间相对位置关系

本文件发布两个静态坐标变换（TF）：
  1. base_link  → radar_link    （毫米波雷达相对车体中心的位置）
  2. base_link  → camera_link   （摄像头相对车体中心的位置）

坐标系约定（ROS REP-105）：
  - base_link：车体中心，x=车头方向，y=左侧，z=向上
  - radar_link：ARS408 雷达安装位置
  - camera_link：USB 摄像头光学中心

安装尺寸（当前车体实测值）：
  雷达：  前方 0.10m，比车体中心高 0.30m
  摄像头：前方 0.15m，与车体中心同高

注意：
  - 此脚本独立使用（在 ars408_radar.launch 中）。
  - 在完整的 radar_obstacle.launch 中，为避免 camera_link 重复
    导致 TF 树冲突，改用 ROS 自带的 static_transform_publisher
    只发布 radar_link 而不发布 camera_link（camera_link 已在
    robot_model_visualization.launch 中定义）。
"""

import rospy
import tf2_ros
import geometry_msgs.msg


def publish_static_tf():
    rospy.init_node('sensor_tf_publisher')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    # ================================================================
    # TF1: base_link → radar_link
    #   物理安装位置：车头前方 10cm，抬高 30cm
    #   无旋转（雷达朝前安装，与车头同向）
    # ================================================================
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = 'base_link'
    t1.child_frame_id = 'radar_link'
    t1.transform.translation.x = 0.10   # 车前 10cm
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.30   # 抬高 30cm（比摄像头高）
    t1.transform.rotation.w = 1.0       # 无旋转
    transforms.append(t1)

    # ================================================================
    # TF2: base_link → camera_link
    #   物理安装位置：车头前方 15cm，与车体同高
    # ================================================================
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = 'base_link'
    t2.child_frame_id = 'camera_link'
    t2.transform.translation.x = 0.15   # 车前 15cm
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0    # 与车体同高
    t2.transform.rotation.w = 1.0
    transforms.append(t2)

    # 一次性发布所有静态 TF
    broadcaster.sendTransform(transforms)

    rospy.loginfo("TF 发布完成: radar_link 相对 camera_link: x=-0.05m, z=+0.30m")
    rospy.loginfo("  radar  = base_link (x=%.2f, y=%.2f, z=%.2f)",
                  t1.transform.translation.x,
                  t1.transform.translation.y,
                  t1.transform.translation.z)
    rospy.loginfo("  camera = base_link (x=%.2f, y=%.2f, z=%.2f)",
                  t2.transform.translation.x,
                  t2.transform.translation.y,
                  t2.transform.translation.z)

    # 保持节点运行（静态 TF 发布后不能退出）
    rospy.spin()


if __name__ == '__main__':
    publish_static_tf()
