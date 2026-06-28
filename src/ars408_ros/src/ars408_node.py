#!/usr/bin/env python3
# coding=utf-8
"""
ARS408-21 毫米波雷达 ROS 驱动节点

协议背景：
  大陆 ARS408-21 是一款 77GHz 长距毫米波雷达，通过 CAN bus 输出检测目标。
  本节点监听 CAN 桥接后的 ROS 话题 /received_messages，解析两帧 CAN 消息：

    - 0x600 (CLUSTER_STATUS)：簇状态包，通知本轮有多少个目标要发送。
      数据 bytes: [num_clusters_low, num_clusters_high, ...]
      总簇数 = data[0] + data[1]（低字节 + 高字节），上限 250。

    - 0x701 (CLUSTER_GENERAL)：簇通用信息包，每个目标占一帧。
      数据 bytes: [ID, b1, b2, b3, b4, b5, b6, RCS, ...]
      解析出纵向距离、横向距离、纵向速度、横向速度、RCS。

  流程：收到 0x600 → 清空上一轮 → 计数 num_of_cluster
        → 陆续收到 n 个 0x701 → 解析并存入 self.clusters{}
        → 当收到数 >= num_of_cluster 时自动发布

  发布话题：
    - /radar/targets        (MarkerArray)   —— RViz 可视化球体
    - /radar/raw            (Float32MultiArray) —— 供融合节点使用
    - /radar/pointcloud     (PointCloud2)   —— 带时间戳，用于 message_filters 同步
"""

import rospy
import math
from can_msgs.msg import Frame
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg


# ============================================================
# CAN ID 定义（ARS408 协议固定）
# ============================================================
CLUSTER_STATUS  = 0x600   # 簇状态包：通知本轮目标数量
CLUSTER_GENERAL = 0x701   # 簇通用包：每个目标的距离/速度/RCS

# ============================================================
# ARS408 协议 —— 数据解析说明
# ============================================================
# 0x701 数据位 (8 bytes):
#   byte 0:  Cluster-ID                     (0-255)
#   byte 1-2: 纵向距离 dist_long (0.2m/bit, offset -500m)
#     bits: [b1_7..b1_0][b2_7..b2_3] 共 13 bits → (val * 0.2) - 500.0
#   byte 2-3: 横向距离 dist_lat  (0.2m/bit, offset -102.3m)
#     bits: [b2_2..b2_0][b3_7..b3_0] 共 11 bits → (val * 0.2) - 102.3
#   byte 4-5: 纵向相对速度 vrel_long (0.25m/s, offset -128m/s)
#     bits: [b4_7..b4_0][b5_7..b5_6] 共 10 bits → (val * 0.25) - 128.0
#   byte 5-6: 横向相对速度 vrel_lat  (0.25m/s, offset -64m/s)
#     bits: [b5_5..b5_0][b6_7..b6_5] 共 9 bits  → (val * 0.25) - 64.0
#   byte 7:   RCS (0.5dB, offset -64dB)
#     → (val * 0.5) - 64.0
#
# 坐标系：雷达自身坐标系，x=车前方(纵向)，y=车左侧(横向)


class ARS408Node:
    """ARS408 毫米波雷达 ROS 节点"""

    def __init__(self):
        rospy.init_node('ars408_node')

        # 当前帧的簇目标缓存
        self.clusters = {}            # {cluster_id: {x, y, vx, vy, rcs, dist}}
        self.num_of_cluster = 0       # 本轮期望接收的目标总数（来自 0x600）
        self.cluster_counter = 0      # 已接收的 0x701 帧计数

        # ---------- 三个输出话题 ----------
        # 1) MarkerArray：用于 RViz 直接显示彩色球体
        self.pub_markers = rospy.Publisher('/radar/targets', MarkerArray, queue_size=10)
        # 2) Float32MultiArray：平坦数组 [id, x, y, vx, vy, dist, rcs, ...]
        #    供 radar_obstacle 包做障碍物检测
        self.pub_raw = rospy.Publisher('/radar/raw', Float32MultiArray, queue_size=10)
        # 3) PointCloud2：带时间戳的点云格式，便于用 message_filters 做时间同步
        self.pub_cloud = rospy.Publisher('/radar/pointcloud', PointCloud2, queue_size=10)

        # 从 socketcan_bridge 接收 CAN 帧（/received_messages）
        rospy.Subscriber('/received_messages', Frame, self.can_callback)

        rospy.loginfo("ARS408 节点启动 (cluster模式 + 时间戳点云) ...")

    # ================================================================
    # CAN 回调
    #   msg.id == 0x600 → 新的一帧开始，先发布上一帧结果，再清空
    #   msg.id == 0x701 → 解析一个目标的数据
    # ================================================================
    def can_callback(self, msg):
        # ---------- 簇状态包 0x600 ----------
        if msg.id == CLUSTER_STATUS and len(msg.data) >= 2:
            # 如果之前已有缓存数据，先把上一帧发布出去
            if self.clusters:
                self.publish()
                self.clusters = {}

            # data[0] 低字节 + data[1] 高字节 = 本轮目标总数
            self.num_of_cluster = min(msg.data[0] + msg.data[1], 250)
            self.cluster_counter = 0

        # ---------- 簇通用包 0x701 ----------
        elif msg.id == CLUSTER_GENERAL and len(msg.data) >= 8:
            self.parse_cluster_general(msg.data)

    # ================================================================
    # 解析 0x701 数据（见头部的比特位说明）
    # ================================================================
    def parse_cluster_general(self, data):
        """
        从 8 bytes 中提取：
          cid        — 簇 ID（0-255）
          dist_long  — 纵向距离 (m)，正=前方
          dist_lat   — 横向距离 (m)，正=左侧
          vrel_long  — 纵向相对速度 (m/s)，正=远离
          vrel_lat   — 横向相对速度 (m/s)，正=左移
          rcs        — 雷达散射截面 (dB)
          dist       — 合成距离 sqrt(x² + y²)
        """
        cid       = data[0]
        # 纵向距离：byte1 全部 8 bit + byte2 高 5 bit = 13 bits
        dist_long = (((data[1] << 5) | (data[2] >> 3)) * 0.2) - 500.0
        # 横向距离：byte2 低 3 bit + byte3 全部 8 bit = 11 bits
        dist_lat  = ((((data[2] & 0x03) << 8) | data[3]) * 0.2) - 102.3
        # 纵向速度：byte4 全部 8 bit + byte5 高 2 bit = 10 bits
        vrel_long = (((data[4] << 2) | (data[5] >> 6)) * 0.25) - 128.0
        # 横向速度：byte5 低 6 bit + byte6 高 3 bit = 9 bits
        vrel_lat  = ((((data[5] & 0x3F) << 3) | (data[6] >> 5)) * 0.25) - 64.0
        # RCS：byte7 * 0.5 - 64
        rcs       = data[7] * 0.5 - 64.0
        # 合成距离
        dist      = math.sqrt(dist_long**2 + dist_lat**2)

        # 存入字典
        self.clusters[cid] = {
            'x': dist_long, 'y': dist_lat,
            'vx': vrel_long, 'vy': vrel_lat,
            'rcs': rcs, 'dist': dist
        }
        self.cluster_counter += 1

        # 当收到的目标数达到预期 → 立即发布并清空
        if self.cluster_counter >= self.num_of_cluster > 0:
            self.publish()
            self.clusters = {}
            self.cluster_counter = 0

    # ================================================================
    # 发布所有三个话题
    # ================================================================
    def publish(self):
        now = rospy.Time.now()
        marker_array = MarkerArray()
        raw_data     = Float32MultiArray()
        points       = []    # PointCloud2 的原始点列表

        for cid, obj in self.clusters.items():
            # 过滤无效距离（0.5m 以内和 200m 以外的杂波/噪声）
            if obj['dist'] < 0.5 or obj['dist'] > 200:
                continue

            # ----- 1. MarkerArray：RViz 红色球体 -----
            # 每个目标显示为半径 0.3m 的橙色半透明球
            m = Marker()
            m.header.frame_id = "radar_link"    # 雷达自身坐标系
            m.header.stamp    = now
            m.ns = "radar"
            m.id = int(cid)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = obj['x']        # 纵向
            m.pose.position.y = obj['y']        # 横向
            m.pose.position.z = 0.5             # 固定抬高 0.5m 便于观察
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 0.9
            m.lifetime = rospy.Duration(0.2)    # 200ms 后自动消失，实现动态刷新
            marker_array.markers.append(m)

            # ----- 2. Float32MultiArray：平坦数组 -----
            # 格式: [cid, x, y, vx, vy, dist, rcs,  cid, x, y, ...]
            # 接收方按每 7 个 float 为一个目标解析
            raw_data.data.extend([
                float(cid), obj['x'], obj['y'],
                obj['vx'], obj['vy'], obj['dist'], obj['rcs']
            ])

            # ----- 3. PointCloud2：带 timestamp 的点云 -----
            # 字段: x, y, z, vx, dist, rcs
            # z 固定为 0.5（同 Marker 高度），方便在 3D 空间观察
            points.append([obj['x'], obj['y'], 0.5,
                           obj['vx'], obj['dist'], obj['rcs']])

        # ---------- 三个发布 ----------
        self.pub_markers.publish(marker_array)

        if raw_data.data:
            self.pub_raw.publish(raw_data)

        # 构建 PointCloud2 消息
        header = std_msgs.msg.Header()
        header.stamp    = now
        header.frame_id = "radar_link"
        fields = [
            PointField('x',    0,  PointField.FLOAT32, 1),
            PointField('y',    4,  PointField.FLOAT32, 1),
            PointField('z',    8,  PointField.FLOAT32, 1),
            PointField('vx',   12, PointField.FLOAT32, 1),
            PointField('dist', 16, PointField.FLOAT32, 1),
            PointField('rcs',  20, PointField.FLOAT32, 1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        self.pub_cloud.publish(cloud)


if __name__ == '__main__':
    ARS408Node()
    rospy.spin()
