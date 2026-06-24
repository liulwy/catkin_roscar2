#!/usr/bin/env python3
"""多点导航脚本：逐点发送目标到 move_base，并按顺序等待完成。"""

import os
import rospy  # ROS Python 客户端库
import actionlib  # ROS action 库，用于与 move_base 等交互
import uuid  # 生成唯一目标ID
import threading
import sys
import yaml
from actionlib_msgs.msg import GoalStatus, GoalID  # action 状态、目标ID
from std_msgs.msg import String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal  # move_base 动作消息
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist  # 位置与姿态消息类型
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
        self.wait_after_arrival = rospy.get_param('~wait_after_arrival', 0.2)
        self.max_retries = rospy.get_param('~max_retries', 2)
        # 修改: 默认为False，上电自动开始
        self.require_enter_to_start = rospy.get_param('~require_enter_to_start', 0)
        self.min_waypoints_to_start = rospy.get_param('~min_waypoints_to_start', 1)

        # 直接发布 move_base/goal 话题，确保逐个目标发送逻辑
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

        # [新增] 创建速度发布者，用于强制急停
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 限制最大 waypoint 个数，避免过多点击导致不必要的队列积压
        self.max_waypoints = rospy.get_param('~max_waypoints', 20)
        self.use_rviz_waypoints = rospy.get_param('~use_rviz_waypoints', 0) # 修改: 默认不依懒Rviz
        self.waypoint_queue = []

        # ==========================================
        # 从 YAML 路径文件加载目标点（替代硬编码）
        # ==========================================
        self._load_waypoints_from_file()

        rospy.loginfo('[%s] 已加载 %d 个目标点', self.ns, len(self.waypoint_queue))
        
        self.current_index = 0
        self.start_event = threading.Event()

        if self.use_rviz_waypoints:
            # 订阅 rviz 2D Navigation 目标输入 topic
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._rviz_goal_callback)
            rospy.loginfo('[%s] 已启用 rviz 点击目标输入. max_waypoints=%d', self.ns, self.max_waypoints)


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

        # 初始化红绿灯相关状态
        self.current_light_color = "none"
        self.light_distance = float('inf')
        self.prev_red_distance = None
        self.is_waiting_for_light = False
        self.traffic_light_sub = None

        # 定位收敛状态：等待 wait_for_convergence 发来 "ready" 信号
        self.localization_ready = False

        if self.require_enter_to_start:
            self._start_enter_listener()
        else:
            self.start_event.set()
            self._subscribe_traffic_light()

    def _subscribe_traffic_light(self):
        """在导航开始后订阅红绿灯结果，避免过早触发停车逻辑。"""
        if self.traffic_light_sub is None:
            self.traffic_light_sub = rospy.Subscriber("/traffic_light_status", String, self.traffic_light_callback, queue_size=1)
            rospy.loginfo('[%s] 已订阅 /traffic_light_status', self.ns)

    def _load_waypoints_from_file(self):
        """从 YAML 路径文件加载目标点，优先使用 rosparam 指定的文件。"""
        path_file = rospy.get_param('~path_file', '')
        if path_file and os.path.exists(path_file):
            try:
                with open(path_file, 'r') as f:
                    data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    for wp in data['waypoints']:
                        self.waypoint_queue.append({
                            'x': float(wp['x']),
                            'y': float(wp['y']),
                            'yaw': float(wp['yaw']),
                        })
                    rospy.loginfo('[%s] 从文件加载路径: %s (%d 点)',
                                  self.ns, path_file, len(self.waypoint_queue))
                    return
            except Exception as e:
                rospy.logwarn('[%s] 路径文件加载失败: %s，回退到参数服务器', self.ns, e)

        # 回退：从 ~waypoints 参数加载
        fallback = rospy.get_param('~waypoints', [])
        if fallback:
            for wp in fallback:
                self.waypoint_queue.append({
                    'x': float(wp['x']),
                    'y': float(wp['y']),
                    'yaw': float(wp['yaw']),
                })
            rospy.loginfo('[%s] 从参数服务器加载 %d 个目标点', self.ns, len(fallback))

    def _start_enter_listener(self):
        """启动后台线程，等待用户按下 Enter 后开始导航。"""
        listener = threading.Thread(target=self._wait_for_enter_to_start, daemon=True)
        listener.start()

    def traffic_light_callback(self, msg):
        """处理红绿灯识别结果，判断距离并控制小车启停。"""

        # 定位收敛信号：由 wait_for_convergence 节点发送
        if msg.data == "ready":
            if not self.localization_ready:
                self.localization_ready = True
                rospy.loginfo('[%s] 收到定位就绪信号，可以开始导航', self.ns)
            return

        parts = msg.data.split(',')
        if len(parts) >= 2:
            self.current_light_color = parts[0]
            dist = float(parts[1])
            if dist < 0:
                self.light_distance = float('inf')
            else:
                if self.current_light_color == "red" and self.light_distance != float('inf'):
                    self.prev_red_distance = self.light_distance
                self.light_distance = dist
            rospy.loginfo_throttle(1.0, "[%s] %s灯 距离: %.2fm", self.ns, self.current_light_color, self.light_distance)
        else:
            self.current_light_color = "none"
            self.light_distance = float('inf')

        # === 核心逻辑：小于1m（精确校验）且为 红灯 ===

        red_hit = False
        if self.current_light_color == "red" and self.light_distance < 1.3:
            if self.prev_red_distance is not None and abs(self.light_distance - self.prev_red_distance) < 0.5:
                red_hit = True
        else:
            # 非红灯时重置缓存，避免连贯性乱序
            self.prev_red_distance = None

        if red_hit:
            if not self.is_waiting_for_light:
                rospy.logwarn("[%s] 距离红灯 %.2fm, 触发停车!", self.ns, self.light_distance)
                self.is_waiting_for_light = True
                
                # 1. 取消规划路径
                self.client.cancel_all_goals()
                
                # 2. [新增] 强制发布 0 速度，覆盖 move_base 的减速过程，实现急停
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                # 连续发送几次确保接收
                for _ in range(3):
                    self.cmd_vel_pub.publish(stop_twist)
                    rospy.sleep(0.02)
                    
        elif self.current_light_color == "green":
            if self.is_waiting_for_light:
                rospy.loginfo("[%s] 检测到绿灯，恢复行驶", self.ns)
            self.is_waiting_for_light = False

    def _wait_for_enter_to_start(self):
        """等待终端 Enter 输入；若 stdin 不可用则自动开始。"""
        if not sys.stdin or not sys.stdin.isatty():
            rospy.logwarn('[%s] stdin 不可用，自动开始导航。', self.ns)
            self.start_event.set()
            self._subscribe_traffic_light()
            return

        rospy.loginfo('[%s] 请先在 RViz 打点，完成后在本终端按 Enter 开始导航。', self.ns)
        try:
            input()
            rospy.loginfo('[%s] 已收到 Enter，开始按队列执行导航。当前队列=%d', self.ns, len(self.waypoint_queue))
            self.start_event.set()
            self._subscribe_traffic_light()
        except EOFError:
            rospy.logwarn('[%s] 读取 Enter 失败(EOF)，自动开始导航。', self.ns)
            self.start_event.set()
            self._subscribe_traffic_light()

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
        """发布目标到 move_base/goal 并等待当前目标完成。加入红绿灯阻塞逻辑。"""
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

        # 把原本的 for 循环改为 while 循环，方便遇到红灯时重试次数不累加
        attempt = 1
        while attempt <= self.max_retries + 1:
            if rospy.is_shutdown():
                return False

            # 1. 发送目标前：如果是红灯，阻塞等待绿灯（或者红灯移出视野）
            while self.is_waiting_for_light and not rospy.is_shutdown():
                rospy.loginfo_throttle(2.0, '[%s] 前方红灯，等待绿灯后再出发...', self.ns)
                rospy.sleep(0.5)

            rospy.loginfo('[%s] 等待 move_base 完成目标 #%s (attempt %d)...',
                          self.ns, self.current_index + 1, attempt)

            self.client.send_goal(goal)
            
            # 2. 移动中阻塞等待：将一次性的120秒阻塞改为0.2秒一轮的循环，以便及时响应红灯
            finished = False
            timeout_time = rospy.Time.now() + rospy.Duration(120.0)
            
            while not rospy.is_shutdown() and rospy.Time.now() < timeout_time:
                # 每次只阻塞 0.2 秒验证成果
                if self.client.wait_for_result(rospy.Duration(0.2)):
                    finished = True
                    break
                
                # 如果等待过程中触发了红灯回调 (回调中已执行 cancel_all_goals，小车正在刹车)
                if self.is_waiting_for_light:
                    rospy.loginfo_throttle(2.0, '[%s] 移动中遇到红灯，停车！', self.ns)
                    break
            
            state = self.client.get_state()

            # 3. 处理红灯被打断的情况
            if self.is_waiting_for_light:
                # 不计入 attempt 失败，直接 continue 进入下一次循环死等绿灯重发
                rospy.loginfo('[%s] 目标 #%s 因为红灯暂停，等待绿灯后自动重新尝试。', self.ns, self.current_index + 1)
                continue

            # 4. 正常返回和异常重试判断
            if not finished:
                rospy.logwarn('[%s] 目标 #%s 超时，取消并重试...', self.ns, self.current_index + 1)
                self.client.cancel_goal()
            elif state == GoalStatus.SUCCEEDED:
                rospy.loginfo('[%s] 目标 #%s 已到达', self.ns, self.current_index + 1)
                rospy.sleep(self.wait_after_arrival)  # 到达后等待，可缓冲小车动作
                return True
            else:
                rospy.logwarn('[%s] 目标 #%s 未达成，状态=%d，重试 %d/%d',
                              self.ns, self.current_index + 1, state, attempt, self.max_retries + 1)
            
            attempt += 1

        rospy.logerr('[%s] 目标 #%s 最终失败，跳过', self.ns, self.current_index + 1)
        return False

    def run(self):
        """主循环：从 waypoint 队列中逐个取出目标，逐个发送并等待完成。"""
        rospy.loginfo('[%s] MultiWaypointNavigator started', self.ns)

        # 等待定位收敛信号（由 wait_for_convergence 节点发布）
        rospy.loginfo('[%s] 等待定位收敛...', self.ns)
        while not self.localization_ready and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.loginfo('[%s] 定位已收敛，开始导航', self.ns)

        waypoint_index = 0
        while not rospy.is_shutdown():
            if not self.start_event.is_set():
                if len(self.waypoint_queue) < self.min_waypoints_to_start:
                    rospy.loginfo_throttle(2.0,
                                           '[%s] 等待 RViz 目标点... 当前=%d, 至少需要=%d，随后按 Enter 开始',
                                           self.ns, len(self.waypoint_queue), self.min_waypoints_to_start)
                else:
                    rospy.loginfo_throttle(2.0,
                                           '[%s] 已收集 %d 个点，等待 Enter 确认开始导航',
                                           self.ns, len(self.waypoint_queue))
                rospy.sleep(0.1)
                continue

            if not self.waypoint_queue:
                rospy.loginfo('[%s] waypoint 队列为空，等待新的 rviz 点或参数 waypoint', self.ns)
                rospy.sleep(0.5)
                continue

            waypoint_index += 1
            self.current_index = waypoint_index
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