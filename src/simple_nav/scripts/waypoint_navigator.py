#!/usr/bin/env python3
# -- coding: UTF-8 --

import rospy
import actionlib
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String as RosString # 避免与 Python 内置的 string 冲突
from std_msgs.msg import Bool as RosBool   # 避免与 Python 内置的 bool 冲突
import rospkg # 用于查找包路径
import os     # 用于拼接路径

class WaypointNavigator:
    def __init__(self, waypoints_file_path): # 注意这里 __init__ 的参数
        # 初始化ROS节点
        # rospy.init_node('waypoint_navigator_node', anonymous=True) # 移动到 main 函数中，确保只初始化一次

        # 获取路点文件的路径
        self.waypoints_file_path = waypoints_file_path
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            rospy.logerr("未能加载路点。退出程序。")
            return

        # 创建一个action客户端与move_base通信
        # 根据你的launch文件，move_base节点名可能不同，这里使用通用的 'move_base'
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base action 服务 (move_base)...")
        
        server_available = self.client.wait_for_server(rospy.Duration(15.0))
        if not server_available:
            rospy.logerr("Action 服务 'move_base' 在15秒内未响应。退出程序。")
            rospy.signal_shutdown("Action 服务未响应") # 触发ROS关闭
            return
            
        rospy.loginfo("已连接到 move_base action 服务。")

        # 新增：任务发布器和完成订阅器
        self.task_publisher = rospy.Publisher('/task', RosString, queue_size=10)
        rospy.loginfo("已创建 /task 发布器 (std_msgs/String)。")

        self.finished_subscriber = rospy.Subscriber('/finished', RosBool, self.finished_callback)
        rospy.loginfo("已订阅 /finished 主题 (std_msgs/Bool)。")
        
        self.task_finished_flag = False # 用于标记当前非 "nav" 任务是否已完成

    def finished_callback(self, msg):
        """
        回调函数，用于处理 /finished 主题的消息。
        """
        if msg.data: # 如果接收到 True
            rospy.loginfo("接收到 /finished信号 (True)，标记任务完成。")
            self.task_finished_flag = True
        else:
            rospy.loginfo("接收到 /finished信号 (False)，任务未完成。")
            

            # 根据需求，你可能希望在这里也重置 task_finished_flag，
            # 但通常我们只关心 True 来继续
            # self.task_finished_flag = False 

    def load_waypoints(self):
        """从YAML文件加载路点"""
        try:
            with open(self.waypoints_file_path, 'r') as f:
                data = yaml.safe_load(f)
                if 'waypoints' in data and data['waypoints']:
                    # 确保每个路点都有 'task' 字段，如果没有则默认为 'nav'
                    loaded_waypoints = []
                    for i, wp in enumerate(data['waypoints']):
                        if 'task' not in wp:
                            rospy.logwarn(f"路点 {i+1} 缺少 'task' 字段，将默认为 'nav'。")
                            wp['task'] = 'nav' # 默认任务类型
                        loaded_waypoints.append(wp)
                    rospy.loginfo(u"成功从 {} 加载了 {} 个路点。".format(self.waypoints_file_path, len(loaded_waypoints)))
                    return loaded_waypoints
                else:
                    rospy.logerr(u"在文件 {} 中未找到 'waypoints' 列表或列表为空。".format(self.waypoints_file_path))
                    return []
        except FileNotFoundError:
            rospy.logerr(u"路点文件未找到: {}".format(self.waypoints_file_path))
            return []
        except yaml.YAMLError as e:
            rospy.logerr(u"解析YAML文件 {} 时出错: {}".format(self.waypoints_file_path, e))
            return []
        except Exception as e:
            rospy.logerr(u"加载路点时发生意外错误: {}".format(e))
            return []

    def go_to_waypoint(self, waypoint_data):
        """发送单个路点目标给move_base"""
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = waypoint_data['frame_id'] 
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = waypoint_data['pose']['position']['x']
        goal.target_pose.pose.position.y = waypoint_data['pose']['position']['y']
        goal.target_pose.pose.position.z = waypoint_data['pose']['position']['z']

        goal.target_pose.pose.orientation.x = waypoint_data['pose']['orientation']['x']
        goal.target_pose.pose.orientation.y = waypoint_data['pose']['orientation']['y']
        goal.target_pose.pose.orientation.z = waypoint_data['pose']['orientation']['z']
        goal.target_pose.pose.orientation.w = waypoint_data['pose']['orientation']['w']

        task_name_for_log = waypoint_data.get('task', '未定义任务') # 用于日志记录

        rospy.loginfo(u"发送任务 '{}' 的目标点 (坐标系: '{}'):".format(task_name_for_log, goal.target_pose.header.frame_id))
        rospy.loginfo(u"  位置 (x,y,z): {}, {}, {}".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z))
        rospy.loginfo(u"  方向 (x,y,z,w): {}, {}, {}, {}".format(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(u"成功到达任务 '{}' 的路点。".format(task_name_for_log))
            return True
        else:
            status_text = self.client.get_goal_status_text()
            rospy.logwarn(u"未能到达任务 '{}' 的路点。状态: {} - {}".format(task_name_for_log, self.client.get_state(), status_text))
            return False

    def navigate(self):
        """按顺序导航到所有加载的路点，并根据task类型执行操作"""
        if not self.waypoints:
            rospy.loginfo("没有路点可供导航。")
            return

        rospy.loginfo("开始路点导航...")
        for i, waypoint in enumerate(self.waypoints):
            rospy.loginfo(u"\n--- 导航到路点 {}/{} (任务: '{}') ---".format(i + 1, len(self.waypoints), waypoint['task']))
            
            success = self.go_to_waypoint(waypoint)
            
            if not success:
                rospy.logwarn(u"由于路点 {} (任务: '{}') 未能到达，导航中止。".format(i + 1, waypoint['task']))
                break 
            
            # 路点到达成功后，处理任务逻辑
            current_task_type = waypoint.get('task', 'nav') # 获取任务类型，默认为 'nav'

            if current_task_type.lower() != 'nav': # 如果任务不是 "nav" (不区分大小写)
                rospy.loginfo(u"已到达路点，任务类型为 '{}'。发布任务并等待 /finished 信号...".format(current_task_type))
                
                # 重置完成标志
                self.task_finished_flag = False
                
                # 发布任务名称
                rospy.sleep(1)
                task_msg = RosString()
                task_msg.data = current_task_type
                self.task_publisher.publish(task_msg)
                rospy.loginfo(u"已向 /task 主题发布: '{}'".format(current_task_type))

                # 等待 /finished 信号
                rate = rospy.Rate(50) # 50 Hz，检查频率
                while not self.task_finished_flag and not rospy.is_shutdown():
                    rospy.logdebug_throttle(5, u"仍在等待任务 '{}' 的 /finished 信号...".format(current_task_type)) # 每5秒打印一次
                    rate.sleep()
                
                if rospy.is_shutdown():
                    rospy.loginfo("ROS已关闭，导航中止。")
                    break
                
                rospy.loginfo(u"任务 '{}' 已完成 (收到 /finished)。继续下一个路点。".format(current_task_type))
            
            else: # 如果任务是 "nav"
                rospy.loginfo(u"任务类型为 'nav'，直接前往下一个路点。")
            
            # 可选：在前往下一个路点前暂停
            # rospy.sleep(1.0) 

        rospy.loginfo("所有路点导航完成（或已中止）。")


if __name__ == '__main__':
    try:
        # 初始化ROS节点，确保只在主程序中执行一次
        rospy.init_node('waypoint_navigator_node', anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('simple_nav') # 假设包名为 simple_nav

        # 假设 waypoints.yaml 在 simple_nav/param/ 目录下
        waypoints_file = os.path.join(package_path, 'param', 'waypoints.yaml') 
        
        rospy.loginfo(u"将从以下文件加载路点: {}".format(waypoints_file))

        navigator = WaypointNavigator(waypoints_file_path=waypoints_file)
        
        # --- 修改开始 ---
        # 如果 __init__ 因为 client 连接失败而调用了 rospy.signal_shutdown(), 
        # rospy.is_shutdown() 应该很快会变成 True。
        # 或者 __init__ 因为路点加载失败而提前返回，导致 navigator.waypoints 为空。

        if rospy.is_shutdown():
            rospy.loginfo("ROS 已被要求关闭 (可能在 Navigator 初始化期间)。导航中止。")
        elif not navigator.waypoints: # 检查路点是否成功加载
            rospy.logerr("路点未加载 (navigator.waypoints 为空或None)，无法开始导航。")
        # 如果到这里，说明路点加载成功，并且 __init__ 中的 wait_for_server 也成功了 (否则会 shutdown)
        # 因此，可以认为 action client 是准备好的。
        else: 
            rospy.loginfo("导航器初始化似乎成功，开始导航流程。")
            navigator.navigate()


    except rospy.ROSInterruptException:
        rospy.loginfo("导航节点被中断。")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("错误：未能找到 'simple_nav' 包。请确保它在你的 ROS_PACKAGE_PATH 中并且已经编译。")
    except Exception as e:
        rospy.logerr(u"主程序发生未处理的异常: {}".format(e))
        import traceback
        traceback.print_exc() # 打印详细的堆栈跟踪