#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Quaternion # 增加了Quaternion
from nav_msgs.msg import Path
from std_srvs.srv import Empty, EmptyResponse
import tf.transformations # 用于欧拉角和四元数转换
import math

class PathDrawerFollower:
    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node('path_drawer_follower_node', anonymous=False)

        # 2. 获取参数
        #   global_frame: RViz中点击点和机器人位姿所在的全局坐标系，例如 'map' 或 'prior_map'
        #   确保这个坐标系与你的定位系统输出的坐标系一致。
        self.global_frame = rospy.get_param('~global_frame', 'map')
        #   机器人控制相关参数
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)        # 线速度 (m/s)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 0.5) # 角速度上限 (rad/s)
        self.goal_tolerance_dist = rospy.get_param('~goal_tolerance_dist', 0.15) # 到达路径点的距离容忍度 (m)
        self.goal_tolerance_angle = rospy.get_param('~goal_tolerance_angle', 0.1) # 到达路径点后的朝向容忍度 (rad)
        self.controller_frequency = rospy.get_param('~controller_frequency', 10) # 控制循环频率 (Hz)
        self.k_p_angle = rospy.get_param('~k_p_angle', 1.5) # 角度P控制器增益
        self.orient_at_waypoint = rospy.get_param('~orient_at_waypoint', False) # 是否在到达每个路径点后调整朝向

        # 3. 内部变量初始化
        self.path_msg = Path() # 用于发布给RViz显示的路径消息
        self.path_msg.header.frame_id = self.global_frame
        self.waypoints = []         # 存储用户定义的路径点 [(x, y, optional_yaw_if_needed_later)]
        self.current_robot_pose = None # 存储机器人当前位姿 (geometry_msgs/Pose)
        self.current_goal_index = 0 # 当前目标路径点的索引
        self.following_active = False # 标志位，表示是否正在跟随路径
        self.state = "IDLE" # 机器人状态: IDLE, ORIENTING, MOVING_TO_WAYPOINT, FINAL_ORIENTING

        # 4. ROS 发布器
        #   发布用户绘制的路径，供RViz显示
        self.path_publisher = rospy.Publisher('~drawn_path', Path, queue_size=1, latch=True)
        #   发布速度指令给机器人底层控制器
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # 确保这是你的机器人控制器的速度话题

        # 5. ROS 订阅器
        #   订阅RViz中 "Publish Point" 工具发布的点击点
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        #   订阅机器人当前的定位信息。你需要根据你的定位系统修改此话题和消息类型。
        #   假设 /localization 发布的是 geometry_msgs/PoseStamped
        rospy.Subscriber('/localization', PoseStamped, self.robot_pose_callback)
        #   如果你的定位发布的是 nav_msgs/Odometry，回调函数需要修改以提取pose
        #   rospy.Subscriber('/odom', Odometry, self.robot_odometry_callback)

        # 6. ROS 服务
        #   提供服务来开始路径跟随
        self.start_following_service = rospy.Service('~start_following', Empty, self.handle_start_following)
        #   提供服务来清除已绘制的路径并停止跟随
        self.clear_path_service = rospy.Service('~clear_path', Empty, self.handle_clear_path)

        rospy.loginfo("路径绘制/跟随器已初始化。")
        rospy.loginfo("  全局坐标系: %s", self.global_frame)
        rospy.loginfo("  在 RViz 中使用 'Publish Point' 工具在 '%s' 坐标系下点击路径点。", self.global_frame)
        rospy.loginfo("  调用服务 '~start_following' 开始跟随。")
        rospy.loginfo("  调用服务 '~clear_path' 清除路径。")

        # 7. 启动主控制循环的定时器
        rospy.Timer(rospy.Duration(1.0/self.controller_frequency), self.control_loop_callback)

    def clicked_point_callback(self, msg):
        """当在RViz中点击一个点时被调用"""
        if msg.header.frame_id != self.global_frame:
            rospy.logwarn("收到的点击点在坐标系 '%s' 下，但期望的是 '%s'。已忽略。",
                          msg.header.frame_id, self.global_frame)
            return

        if self.following_active:
            rospy.loginfo("当前正在跟随路径。请先调用 '~clear_path' 服务清除当前路径。")
            return

        # 将点击的点添加到路径点列表
        # 注意：这里我们只存储 x, y。如果需要目标朝向，RViz的clicked_point不直接提供yaw，
        # 你可能需要手动为每个点设定或通过其他方式（如两次点击定义一个姿态）。
        # 为简单起见，此版本仅使用 (x, y) 并让机器人在移动时朝向下一个点。
        self.waypoints.append((msg.point.x, msg.point.y))
        rospy.loginfo("添加路径点: (%.2f, %.2f)。当前共 %d 个点。", msg.point.x, msg.point.y, len(self.waypoints))

        # 更新并发布Path消息以在RViz中显示
        self.publish_path_for_rviz()

    def robot_pose_callback(self, msg):
        """当收到机器人位姿信息时被调用 (假设是PoseStamped)"""
        if msg.header.frame_id != self.global_frame:
            rospy.logwarn_throttle(5.0, "收到的机器人位姿在坐标系 '%s' 下，但期望的是 '%s'。请检查TF树或话题源。",
                                   msg.header.frame_id, self.global_frame)
            self.current_robot_pose = None # 标记位姿无效
            return
        self.current_robot_pose = msg.pose # PoseStamped.pose 是 geometry_msgs/Pose

    # 如果你的定位系统发布的是nav_msgs/Odometry，使用这个回调并修改订阅器
    # def robot_odometry_callback(self, msg):
    #     """当收到机器人里程计信息时被调用 (Odometry)"""
    #     if msg.header.frame_id != self.global_frame: # 通常odom的header.frame_id是 'odom'，child_frame_id是 'base_link'
    #                                                 # 如果你的global_frame是 'map'，你需要确保有 'map' -> 'odom' 的TF
    #                                                 # 并且这个回调应该处理的是 map 坐标系下的位姿。
    #                                                 # 对于fastlio，'/localization' 应该已经是 'map' 坐标系下的了。
    #         rospy.logwarn_throttle(5.0, "收到的里程计在坐标系 '%s' 下，但期望的是 '%s'。请检查TF树或话题源。",
    #                                msg.header.frame_id, self.global_frame)
    #         self.current_robot_pose = None
    #         return
    #     self.current_robot_pose = msg.pose.pose # Odometry.pose 是 PoseWithCovariance，.pose.pose 是 Pose

    def publish_path_for_rviz(self):
        """构造并发布nav_msgs/Path消息，供RViz显示"""
        self.path_msg.poses = []
        current_time = rospy.Time.now()
        for i, (x, y) in enumerate(self.waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time
            pose_stamped.header.seq = i
            pose_stamped.header.frame_id = self.global_frame
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            # 对于Path消息，方向通常是可选的，或者指向下一个点。这里简化为默认。
            pose_stamped.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose_stamped)
        self.path_publisher.publish(self.path_msg)

    def handle_start_following(self, req):
        """处理'/start_following'服务请求"""
        if not self.waypoints:
            rospy.logwarn("路径点列表为空，无法开始跟随。")
            return EmptyResponse()
        if self.current_robot_pose is None:
            rospy.logwarn("尚未接收到有效的机器人位姿，无法开始跟随。")
            return EmptyResponse()
        if self.following_active:
            rospy.loginfo("已在跟随路径中。")
            return EmptyResponse()

        self.current_goal_index = 0
        self.following_active = True
        self.state = "ORIENTING" # 开始时先朝向第一个点
        rospy.loginfo("服务调用：开始跟随路径，共 %d 个点。", len(self.waypoints))
        return EmptyResponse()

    def handle_clear_path(self, req):
        """处理'/clear_path'服务请求"""
        rospy.loginfo("服务调用：清除路径并停止跟随。")
        self.waypoints = []
        self.publish_path_for_rviz() # 发布空路径
        self.following_active = False
        self.state = "IDLE"
        self.current_goal_index = 0
        # 发送停止指令
        self.send_stop_command()
        return EmptyResponse()

    def send_stop_command(self):
        """发送停止运动的指令"""
        twist_cmd = Twist() # 默认所有速度为0
        self.cmd_vel_publisher.publish(twist_cmd)

    def get_current_yaw(self):
        """从current_robot_pose获取当前的偏航角(yaw)"""
        if self.current_robot_pose is None:
            return None
        orientation_q = self.current_robot_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def normalize_angle(self, angle):
        """将角度标准化到 [-pi, pi] 区间"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop_callback(self, event):
        """主控制循环，由定时器周期性调用"""
        if not self.following_active or self.current_robot_pose is None:
            # 如果未激活跟随，或者没有机器人位姿，则不执行控制逻辑
            # 但如果激活了却没位姿，可以打印警告
            if self.following_active and self.current_robot_pose is None:
                rospy.logwarn_throttle(2.0, "正在尝试跟随路径，但机器人位姿无效或未收到。")
                self.send_stop_command() # 安全起见，发送停止指令
            return

        # 检查是否已完成所有路径点
        if self.current_goal_index >= len(self.waypoints):
            rospy.loginfo("已完成所有路径点！")
            self.following_active = False
            self.state = "IDLE"
            self.send_stop_command()
            return

        # 获取当前的目标路径点 (x, y)
        target_x, target_y = self.waypoints[self.current_goal_index]
        robot_x = self.current_robot_pose.position.x
        robot_y = self.current_robot_pose.position.y
        current_yaw = self.get_current_yaw()

        if current_yaw is None: # 无法获取当前朝向
            rospy.logwarn_throttle(1.0, "无法获取机器人当前朝向。")
            self.send_stop_command()
            return

        # 计算到目标点的距离和角度差
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx) # 目标点相对于机器人当前位置的角度

        # --- 状态机逻辑 ---
        twist_cmd = Twist()

        if self.state == "ORIENTING":
            # 状态：朝向当前路径点
            angle_diff = self.normalize_angle(angle_to_target - current_yaw)
            if abs(angle_diff) > self.goal_tolerance_angle:
                # 角度误差较大，旋转
                twist_cmd.angular.z = self.k_p_angle * angle_diff
                # 限制最大角速度
                twist_cmd.angular.z = max(-self.angular_speed_limit, min(self.angular_speed_limit, twist_cmd.angular.z))
            else:
                # 朝向完成，切换到移动状态
                self.state = "MOVING_TO_WAYPOINT"
                rospy.loginfo("朝向 waypoint %d 完成，开始移动。", self.current_goal_index)

        elif self.state == "MOVING_TO_WAYPOINT":
            # 状态：向当前路径点移动
            if distance_to_target > self.goal_tolerance_dist:
                # 未到达目标点，继续移动
                angle_diff = self.normalize_angle(angle_to_target - current_yaw)

                # 简单的P控制器控制角速度以保持朝向目标
                # 如果角度偏差过大，优先调整角度；否则，同时前进和微调角度
                if abs(angle_diff) > math.pi / 4: # 例如45度，如果偏差太大，减慢或停止线速度
                    twist_cmd.linear.x = self.linear_speed * 0.3 # 减速以更好地转向
                    twist_cmd.angular.z = self.k_p_angle * angle_diff
                else:
                    twist_cmd.linear.x = self.linear_speed
                    twist_cmd.angular.z = self.k_p_angle * angle_diff * 0.8 # 前进时角度控制稍弱一些

                # 限制最大角速度
                twist_cmd.angular.z = max(-self.angular_speed_limit, min(self.angular_speed_limit, twist_cmd.angular.z))

            else:
                # 到达当前路径点
                rospy.loginfo("到达 waypoint %d: (%.2f, %.2f)。", self.current_goal_index, target_x, target_y)
                self.current_goal_index += 1
                if self.current_goal_index < len(self.waypoints):
                    # 如果还有下一个点，切换到朝向下一个点
                    self.state = "ORIENTING"
                else:
                    # 所有点都完成
                    rospy.loginfo("所有路径点均已到达。")
                    self.state = "IDLE"
                    self.following_active = False
                    # self.send_stop_command() # 在循环末尾统一发送

        # 发布速度指令
        self.cmd_vel_publisher.publish(twist_cmd)
        if self.state == "IDLE" and not self.following_active : # 确保在最终IDLE状态停止
             self.send_stop_command()


if __name__ == '__main__':
    try:
        follower_node = PathDrawerFollower()
        rospy.spin() # 保持节点运行，等待回调
    except rospy.ROSInterruptException:
        rospy.loginfo("路径绘制/跟随节点已关闭。")
    except Exception as e:
        rospy.logerr("节点运行时发生未捕获的异常: %s", str(e))