#!/usr/bin/env python3
"""多点导航脚本：逐点发送目标到 move_base，并按顺序等待完成。"""

import rospy  # ROS Python 客户端库
import actionlib  # ROS action 库，用于与 move_base 等交互
import uuid  # 生成唯一目标ID
from actionlib_msgs.msg import GoalStatus, GoalID  # action 状态、目标ID

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal  # move_base 动作消息
from geometry_msgs.msg import Pose, PoseStamped, Quaternion  # 位置与姿态消息类型
from tf.transformations import quaternion_from_euler, euler_from_quaternion  # 欧拉角<->四元数转换


class MultiWaypointNavigator(object):
    """封装多点导航逻辑的类。"""

    def __init__(self):
        # 获取当前 ROS 命名空间
        self.ns = rospy.get_namespace()

        # 创建 action client，用于向 move_base 发送目标并等待状态
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('[%s] Waiting for move_base action server...', self.ns)
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr('[%s] move_base server not available after 30s', self.ns)
            raise rospy.ROSException('move_base action server not available')

        # 读取参数：waypoints 里是一组点列表，每个点含 x,y,yaw
        self.waypoints = rospy.get_param('~waypoints', [])
        self.arrival_tolerance = rospy.get_param('~arrival_tolerance', 0.3)
        self.wait_after_arrival = rospy.get_param('~wait_after_arrival', 1.0)
        self.max_retries = rospy.get_param('~max_retries', 2)

        # 直接发布 move_base/goal 话题，确保逐个目标发送逻辑
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

        # 限制最大 waypoint 个数，避免过多点击导致不必要的队列积压
        self.max_waypoints = rospy.get_param('~max_waypoints', 20)
        self.use_rviz_waypoints = rospy.get_param('~use_rviz_waypoints', True)
        self.waypoint_queue = []

        if self.use_rviz_waypoints:
            # 订阅 rviz 2D Navigation 目标输入 topic
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._rviz_goal_callback)
            rospy.loginfo('[%s] 已启用 rviz 点击目标输入. max_waypoints=%d', self.ns, self.max_waypoints)

        # 如果参数给了预设的 waypoint 列表，预先加入队列
        if isinstance(self.waypoints, list) and len(self.waypoints) > 0:
            for wp in self.waypoints:
                if len(self.waypoint_queue) >= self.max_waypoints:
                    rospy.logwarn('[%s] waypoint 参数列表超过 max_waypoints (%d)，额外部分忽略', self.ns, self.max_waypoints)
                    break
                self.waypoint_queue.append(wp)

        # 若没有 waypoint 并且未启用 rviz 订阅，则使用默认测试值
        if not self.waypoint_queue and not self.use_rviz_waypoints:
            rospy.logwarn('[%s] 未配置 waypoint 且未启用 rviz 模式，使用默认测试点', self.ns)
            self.waypoint_queue = [
                {'x': 0.5, 'y': 0.0, 'yaw': 0.0},
                {'x': 0.5, 'y': 0.5, 'yaw': 1.57},
                {'x': 0.0, 'y': 0.5, 'yaw': 3.14},
            ]

        rospy.loginfo('[%s] 当前 waypoint 队列长度 %d', self.ns, len(self.waypoint_queue))
        rospy.loginfo('[%s] 已接收 %d 个导航点，小车开始启动。', self.ns, len(self.waypoint_queue))

    def _rviz_goal_callback(self, msg):
        """接收 rviz 投射点（/move_base_simple/goal），并追加到 waypoint 队列。"""
        if len(self.waypoint_queue) >= self.max_waypoints:
            rospy.logwarn('[%s] waypoint 队列已满(max=%d)，忽略 rviz 点击点', self.ns, self.max_waypoints)
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        # 从 quaternion 转 yaw（仅 2D 平面）
        _, _, yaw = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        self.waypoint_queue.append({'x': x, 'y': y, 'yaw': yaw})
        rospy.loginfo('[%s] 收到 rviz 目标点 #%d: x=%.3f y=%.3f yaw=%.3f', self.ns, len(self.waypoint_queue), x, y, yaw)

    def _create_goal(self, x, y, yaw):
        """将 x, y, yaw 转为 MoveBaseGoal。"""
        # 将 yaw (rad) 转为四元数（仅绕 z 轴旋转）
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'  # 使用全局 map 坐标系
        goal.target_pose.header.stamp = rospy.Time.now()

        # 目标位置设置
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # 目标朝向设置
        goal.target_pose.pose.orientation = Quaternion(*quaternion)

        return goal

    def send_goal(self, x, y, yaw):
        """发布目标到 move_base/goal 并等待当前目标完成。"""
        goal = self._create_goal(x, y, yaw)

        # 构造 MoveBaseActionGoal，包装目标和 GoalID
        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal_id = GoalID()
        action_goal.goal_id.stamp = rospy.Time.now()
        action_goal.goal_id.id = str(uuid.uuid4())  # 生成唯一ID，便于后续追踪
        action_goal.goal = goal

        rospy.loginfo('[%s] 向 move_base/goal 发布目标 #%s: x=%.3f y=%.3f yaw=%.3f',
                      self.ns, self.current_index + 1, x, y, yaw)
        self.goal_pub.publish(action_goal)

        # 每个目标允许多次重试
        for attempt in range(1, self.max_retries + 2):
            if rospy.is_shutdown():
                return False

            rospy.loginfo('[%s] 等待 move_base 完成目标 #%s (attempt %d)...',
                          self.ns, self.current_index + 1, attempt)

            # 这里使用 move_base action client 监测目标状态完成情况
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration(120.0))
            state = self.client.get_state()

            if not finished:
                # 超时：认为当前目标未完成，取消并尝试重发
                rospy.logwarn('[%s] 目标 #%s 超时，取消并重试...', self.ns, self.current_index + 1)
                self.client.cancel_goal()
            elif state == GoalStatus.SUCCEEDED:
                # 成功：目标完成，返回 True 继续下一个目标
                rospy.loginfo('[%s] 目标 #%s 已到达', self.ns, self.current_index + 1)
                rospy.sleep(self.wait_after_arrival)  # 到达后等待，可缓冲小车动作
                return True
            else:
                # 失败：非成功状态，记录并根据重试次数决定是否继续
                rospy.logwarn('[%s] 目标 #%s 未达成，状态=%d，重试 %d/%d',
                              self.ns, self.current_index + 1, state, attempt, self.max_retries + 1)

        rospy.logerr('[%s] 目标 #%s 最终失败，跳过', self.ns, self.current_index + 1)
        return False

    def run(self):
        """主循环：从 waypoint 队列中逐个取出目标，逐个发送并等待完成。"""
        rospy.loginfo('[%s] MultiWaypointNavigator started', self.ns)

        waypoint_index = 0
        while not rospy.is_shutdown():
            if not self.waypoint_queue:
                rospy.loginfo('[%s] waypoint 队列为空，等待新的 rviz 点或参数 waypoint', self.ns)
                rospy.sleep(0.5)
                continue

            waypoint_index += 1
            wp = self.waypoint_queue.pop(0)
            x = float(wp.get('x', 0.0))
            y = float(wp.get('y', 0.0))
            yaw = float(wp.get('yaw', 0.0))

            rospy.loginfo('[%s] Navigating to waypoint %d: (%.3f, %.3f, %.3f)',
                          self.ns, waypoint_index, x, y, yaw)

            success = self.send_goal(x, y, yaw)
            if not success:
                rospy.logwarn('[%s] 目标 #%d 导航失败，继续下一个', self.ns, waypoint_index)
                continue

            rospy.loginfo('[%s] 目标 #%d 完成', self.ns, waypoint_index)


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('multi_waypoint_nav', anonymous=False)

    # 构造导航对象并运行
    navigator = MultiWaypointNavigator()
    try:
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('[%s] ROS Interrupt received. stopping.', navigator.ns)
        # 收到中断时取消当前 move_base 目标
        navigator.client.cancel_goal()
