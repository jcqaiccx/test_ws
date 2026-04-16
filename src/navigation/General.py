#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, rospy, time
from std_msgs.msg import Float64
from std_msgs.msg import String


class General():
    def __init__(self):
        """
        中控节点（本文件）负责各功能节点间的相互通信，各功能节点均发送消息到中控并接收中控返回的消息
        """
        self.pub_nav = rospy.Publisher('navigation', String, queue_size=10)
        self.pub_map = rospy.Publisher('map', String, queue_size=10)
        self.pub_pid = rospy.Publisher('pid', String, queue_size=10)
        self.pub_arm = rospy.Publisher('arm', String, queue_size=10)
        self.pub_vision = rospy.Publisher('vision', String, queue_size=10)

        rospy.Subscriber("arm_back", String, self.arm_back)
        rospy.Subscriber("navigation_back", String, self.navigation_back)
        rospy.Subscriber("map_back", String, self.map_back)
        rospy.Subscriber("vision_back", String, self.vision_back)
        rospy.Subscriber("pid_back", String, self.pid_back)

    """
    （在另外的文件中实现）视觉节点要实现的是订阅vision话题（接收来自中控的消息），并能向vision_back话题中发送消息（向中控发送消息）
    当识别到袋子时，一方面直接向pid节点发送袋子坐标，另一方面向中控节点（本文件）发送bag消息
    当识别到人做出停止跟随的手势时，向中控节点发送stop消息
    """
    def vision_back(self, msg):
        # 1.视觉开始识别，若识别到袋子，则视觉节点发送bag消息到vision_back话题
        if msg.data == "left":
            print("识别到左边袋子，开始PID_follow_bag")
            # 发送消息到PID节点
            self.pub_pid.publish("follow_bag")
        if msg.data == "right":
            print("识别到右边袋子，开始PID_follow_bag")
            # 发送消息到PID节点
            self.pub_pid.publish("follow_bag")
        # 4.视觉持续识别，若识别到停止手势，则视觉节点发送stop消息到vision_back话题
        if msg.data == "stop":
            print("识别到停止手势，停止跟随人")
            # 发送消息到PID节点
            self.pub_pid.publish("pid_stop")

    """
    （在另外的文件中实现）pid节点要实现的是订阅pid话题（接收来自中控的消息），并能向pid_back话题中发送消息（向中控发送消息）
    当接收到来自中控的follow_bag消息时，开始去到袋子跟前，并且在到达袋子跟前后向中控发送follow_bag_finish消息
    当接收到来自中控的follow_people消息时，开始跟随人
    当接收到来自中控的pid_stop消息后，停止跟随人，并且向中控发送follow_people_finish消息
    """
    def pid_back(self, msg):
        # 2.PID到达袋子跟前，pid节点发送follow_bag_finish到pid_back话题
        if msg.data == "follow_bag_finish":
            print("到达袋子跟前，开始机械臂抓取")
            # 发送消息到arm节点
            self.pub_arm.publish("grasp")
        # 5.PID停止跟随人后，pid节点发送follow_people_finish消息到pid_back话题
        if msg.data == "follow_people_finish":
            print("PID停止跟随人，开始机械臂松手并保存地图")
            # 发送消息到arm节点和map节点
            self.pub_arm.publish("give")
            self.pub_map.publish("save_map")

    """
    （在另外的文件中实现）arm节点要实现的是订阅arm话题（接收来自中控的消息），并能向arm_back话题中发送消息（向中控发送消息）
    当接收到来自中控的grasp消息时，开始机械臂抓取，并且在机械臂抓取结束后向中控发送grasp_finish消息
    当接收到来自中控的give消息时，开始机械臂松手，并且在机械臂松手结束后向中控发送give_finish消息
    """
    def arm_back(self, msg):
        # 3.机械臂抓取完毕，arm节点发送grasp_finish消息到arm_back话题
        if msg.data == "grasp_finish":
            print("机械臂抓取完毕，开始PID跟随人并建图")
            # 发送消息到pid节点和map节点
            self.pub_pid.publish("follow_people")
            self.pub_nav.publish("start_nav")
            self.pub_map.publish("start_map")
        # 6.机械臂松手完毕，arm节点发送give_finish消息到arm_back话题
        if msg.data == "give_finish":
            print("机械臂松手完毕，开始导航返回起点")
            # 发送消息到导航节点
            self.pub_nav.publish("nav_back")

    """
    （在另外的文件中实现）导航节点要实现的是订阅navigation话题（接收来自中控的消息），并能向nav_back话题中发送消息（向中控发送消息）
    当接收到来自中控的nav_back消息时，开始导航返回，导航时采用刚刚建好并保存的地图
    """
    def navigation_back(self, msg):
        if msg.data == "nav_complete":
            print("导航返回起点完成")   
        
    """
    （在另外的文件中实现）mymap节点要实现的是订阅mymap话题（接收来自中控的消息），并能向map_back话题中发送消息（向中控发送消息）
    当接收到来自中控的start_build_map消息时，开始建图
    当接收到来自中控的save_map消息时，保存地图
    """
    def map_back(self, msg):
        if msg.data == "map_saved":
            print("地图保存成功")
        elif msg.data == "build_map_complete":
            print("实时建图完成")
            
            
            
if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    P = General()
    rospy.spin()