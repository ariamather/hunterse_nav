#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # 修改 LaserScan 消息中的 frame_id
    msg.header.frame_id = "laser_link_2d"  # 设置你想要的 frame_id
    # 发布修改后的消息
    scan_pub.publish(msg)

def scan_relay_node():
    # 初始化节点
    rospy.init_node('scan_relay_node', anonymous=True)

    # 创建一个发布者，发布到新的话题
    global scan_pub
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    # 创建一个订阅者，订阅 /scan 消息
    rospy.Subscriber('/scan_test', LaserScan, scan_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        scan_relay_node()
    except rospy.ROSInterruptException:
        pass
