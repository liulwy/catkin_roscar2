#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String

def main():
    # 初始化ROS节点
    rospy.init_node("traffic_light_yolo_node")
    # 创建发布者，发布交通灯状态
    pub = rospy.Publisher("/traffic_light_state", String, queue_size=10)
    # 设置循环频率10Hz
    rate = rospy.Rate(10)

    # 节点主循环
    while not rospy.is_shutdown():
        msg = "traffic light detection running"
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        # 捕获ROS中断异常，优雅退出
        rospy.loginfo("Traffic light yolo node shutdown.")