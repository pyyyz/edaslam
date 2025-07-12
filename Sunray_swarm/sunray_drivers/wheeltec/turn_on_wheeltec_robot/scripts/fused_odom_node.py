#!/usr/bin/env python3
# coding=utf-8

import sys
if '/opt/ros/noetic/lib/python3/dist-packages' not in sys.path:
    sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import threading

class FusedOdomNode:
    def __init__(self):
        rospy.init_node('fused_odometry_node')
        
        # 参数配置
        self.ekf_topic = rospy.get_param("~ekf_topic", "/robot_pose_ekf/odom_combined")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/odom_fused")
        
        # 初始化存储容器
        self.latest_ekf_pose = None
        self.latest_odom_twist = None
        self.latest_ekf_time = rospy.Time(0)
        self.latest_odom_time = rospy.Time(0)
        
        # 线程锁保证数据同步
        self.lock = threading.Lock()
        
        # 创建发布器
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        
        # 创建订阅器
        rospy.Subscriber(self.ekf_topic, PoseWithCovarianceStamped, self.ekf_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # 定时器用于合成和发布消息
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_fused_odom)  # 50Hz
        
        rospy.loginfo("融合里程计节点已启动")
        rospy.loginfo("融合位姿源: %s", self.ekf_topic)
        rospy.loginfo("速度源: %s", self.odom_topic)
        rospy.loginfo("输出话题: %s", self.output_topic)
    
    def ekf_callback(self, msg):
        """处理robot_pose_ekf的输出"""
        with self.lock:
            # 保存最新的融合位姿
            self.latest_ekf_pose = msg
            self.latest_ekf_time = rospy.Time.now()
    
    def odom_callback(self, msg):
        """处理原始里程计的速度信息"""
        with self.lock:
            # 保存最新的速度信息
            self.latest_odom_twist = msg.twist
            self.latest_odom_time = rospy.Time.now()
    
    def publish_fused_odom(self, event):
        """定时发布融合后的里程计信息"""
        with self.lock:
            # 检查是否有足够的新数据
            current_time = rospy.Time.now()
            
            # 检查数据时效性
            ekf_age = (current_time - self.latest_ekf_time).to_sec() if self.latest_ekf_pose else float('inf')
            odom_age = (current_time - self.latest_odom_time).to_sec() if self.latest_odom_twist else float('inf')
            
            # 如果任一数据过于陈旧，跳过此次发布
            if ekf_age > 0.5 or odom_age > 0.5:
                rospy.logwarn_once("缺少数据或数据过期: EKF=%.2fs, Odom=%.2fs", 
                                 ekf_age if ekf_age < float('inf') else -1, 
                                 odom_age if odom_age < float('inf') else -1)
                return
            
            # 创建新的里程计消息
            fused_odom = Odometry()
            
            # 使用融合位姿的时间戳
            fused_odom.header.stamp = self.latest_ekf_pose.header.stamp
            fused_odom.header.frame_id = self.latest_ekf_pose.header.frame_id
            
            # 设置位姿信息
            fused_odom.child_frame_id = "base_footprint"  # 根据实际情况调整
            
            # 使用融合后的位姿
            fused_odom.pose = self.latest_ekf_pose.pose
            
            # 使用原始里程计的速度信息
            fused_odom.twist = self.latest_odom_twist
            
            # 调整速度协方差以反映时间差异
            self.adjust_twist_covariance(fused_odom)
            
            # 发布融合后的里程计
            self.odom_pub.publish(fused_odom)
    
    def adjust_twist_covariance(self, odom_msg):
        """调整速度协方差以反映时间差异"""
        time_diff = abs((self.latest_ekf_time - self.latest_odom_time).to_sec())
        
        # 最大容忍时间差阈值 (100ms)
        max_time_diff = 0.1
        
        if time_diff > max_time_diff:
            # 线性增加协方差误差
            scale_factor = min(1.0 + (time_diff - max_time_diff) * 2, 5.0)
            
            # 应用缩放因子到线性速度协方差
            for i in [0, 7, 14]:
                odom_msg.twist.covariance[i] *= scale_factor
            
            # 应用缩放因子到角速度协方差
            for i in [21, 28, 35]:
                odom_msg.twist.covariance[i] *= scale_factor
            
            rospy.logdebug("速度协方差调整因子: %.2f (时间差=%.3fs)", scale_factor, time_diff)
    
    def run(self):
        """主运行循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = FusedOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("融合里程计节点已关闭")
    except Exception as e:
        rospy.logerr("节点运行异常: %s", str(e))