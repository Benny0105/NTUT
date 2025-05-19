#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import threading
import time
import rospkg

class PathPlotter:
    def __init__(self):
        rospy.init_node('path_plotter', anonymous=True)
        
        # 初始化數據鎖用於線程安全
        self.data_lock = threading.Lock()
        
        # 獲取ROS包路徑解析器
        rospack = rospkg.RosPack()
        
        # 獲取參數
        self.save_path = rospy.get_param('~save_path', '')
        self.max_points = int(rospy.get_param('~max_points', 1000))
        self.update_rate = float(rospy.get_param('~update_rate', 1.0))  # Hz
        
        # 確保保存路徑存在
        save_dir = os.path.dirname(self.save_path)
        if not os.path.exists(save_dir):
            try:
                os.makedirs(save_dir)
            except OSError as e:
                rospy.logwarn("無法創建保存目錄: %s - %s", save_dir, str(e))
                # 回退到ekf包的logs目錄
                try:
                    ekf_path = rospack.get_path('ekf')
                    self.save_path = os.path.join(ekf_path, 'logs', 'ekf_path.png')
                    logs_dir = os.path.join(ekf_path, 'logs')
                    if not os.path.exists(logs_dir):
                        os.makedirs(logs_dir)
                    rospy.loginfo("將保存到: %s", self.save_path)
                except Exception as e2:
                    rospy.logwarn("無法創建日誌目錄: %s", str(e2))
                    # 最終回退到當前工作目錄
                    self.save_path = os.path.join(os.getcwd(), 'ekf_path.png')
                    rospy.loginfo("將保存到: %s", self.save_path)
        
        # 初始化路徑存儲
        self.truth_path = {'x': [], 'y': [], 'z': []}
        self.ekf_path = {'x': [], 'y': [], 'z': []}
        
        # 訂閱話題
        self.truth_sub = rospy.Subscriber('/ground_truth/pose', PoseWithCovarianceStamped, self.truth_callback)
        
        # 嘗試訂閱EKF輸出話題 - 支持不同消息類型
        try:
            self.ekf_sub = rospy.Subscriber('/px4_style_ekf/pose', PoseWithCovarianceStamped, self.ekf_callback_with_cov)
            rospy.loginfo("訂閱 /px4_style_ekf/pose (PoseWithCovarianceStamped)")
        except Exception as e:
            rospy.logwarn("無法訂閱 /px4_style_ekf/pose 作為 PoseWithCovarianceStamped: %s", str(e))
            try:
                self.ekf_sub = rospy.Subscriber('/px4_style_ekf/pose', PoseStamped, self.ekf_callback)
                rospy.loginfo("訂閱 /px4_style_ekf/pose (PoseStamped)")
            except Exception as e:
                rospy.logwarn("無法訂閱 /px4_style_ekf/pose 作為 PoseStamped: %s", str(e))
                rospy.loginfo("嘗試訂閱 /px4_style_ekf/odom 作為 Odometry")
                self.ekf_sub = rospy.Subscriber('/px4_style_ekf/odom', Odometry, self.ekf_callback_odom)
        
        # 創建定時器更新繪圖
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update_plot)
        
        rospy.loginfo("Path Plotter已初始化，將保存到: %s", self.save_path)
    
    def truth_callback(self, msg):
        with self.data_lock:
            self.truth_path['x'].append(msg.pose.pose.position.x)
            self.truth_path['y'].append(msg.pose.pose.position.y)
            self.truth_path['z'].append(msg.pose.pose.position.z)
            
            # 限制點數
            if len(self.truth_path['x']) > self.max_points:
                self.truth_path['x'] = self.truth_path['x'][-self.max_points:]
                self.truth_path['y'] = self.truth_path['y'][-self.max_points:]
                self.truth_path['z'] = self.truth_path['z'][-self.max_points:]
    
    def ekf_callback(self, msg):
        with self.data_lock:
            self.ekf_path['x'].append(msg.pose.position.x)
            self.ekf_path['y'].append(msg.pose.position.y)
            self.ekf_path['z'].append(msg.pose.position.z)
            
            # 限制點數
            if len(self.ekf_path['x']) > self.max_points:
                self.ekf_path['x'] = self.ekf_path['x'][-self.max_points:]
                self.ekf_path['y'] = self.ekf_path['y'][-self.max_points:]
                self.ekf_path['z'] = self.ekf_path['z'][-self.max_points:]
    
    def ekf_callback_with_cov(self, msg):
        with self.data_lock:
            self.ekf_path['x'].append(msg.pose.pose.position.x)
            self.ekf_path['y'].append(msg.pose.pose.position.y)
            self.ekf_path['z'].append(msg.pose.pose.position.z)
            
            # 限制點數
            if len(self.ekf_path['x']) > self.max_points:
                self.ekf_path['x'] = self.ekf_path['x'][-self.max_points:]
                self.ekf_path['y'] = self.ekf_path['y'][-self.max_points:]
                self.ekf_path['z'] = self.ekf_path['z'][-self.max_points:]
    
    def ekf_callback_odom(self, msg):
        with self.data_lock:
            self.ekf_path['x'].append(msg.pose.pose.position.x)
            self.ekf_path['y'].append(msg.pose.pose.position.y)
            self.ekf_path['z'].append(msg.pose.pose.position.z)
            
            # 限制點數
            if len(self.ekf_path['x']) > self.max_points:
                self.ekf_path['x'] = self.ekf_path['x'][-self.max_points:]
                self.ekf_path['y'] = self.ekf_path['y'][-self.max_points:]
                self.ekf_path['z'] = self.ekf_path['z'][-self.max_points:]
    
    def update_plot(self, event):
        try:
            with self.data_lock:
                if not (self.truth_path['x'] and self.ekf_path['x']):
                    return  # 無數據可繪製
                
                # 創建圖形
                plt.figure(figsize=(12, 10))
                
                # 2D路徑 (俯視圖)
                plt.subplot(2, 1, 1)
                plt.plot(self.truth_path['x'], self.truth_path['y'], 'b-', label='Ground Truth')
                plt.plot(self.ekf_path['x'], self.ekf_path['y'], 'r-', label='EKF Estimation')
                plt.grid(True)
                plt.axis('equal')
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                plt.title('2D Path (Top View)')
                plt.legend()
                
                # 高度變化
                plt.subplot(2, 1, 2)
                
                # 創建時間索引
                truth_indices = np.arange(len(self.truth_path['x']))
                ekf_indices = np.arange(len(self.ekf_path['x']))
                
                plt.plot(truth_indices, self.truth_path['z'], 'b-', label='Ground Truth')
                plt.plot(ekf_indices, self.ekf_path['z'], 'r-', label='EKF Estimation')
                plt.grid(True)
                plt.xlabel('Time Index')
                plt.ylabel('Z (m)')
                plt.title('Height vs Time')
                plt.legend()
                
                plt.tight_layout()
                
                # 保存圖形
                try:
                    plt.savefig(self.save_path)
                    rospy.loginfo("已保存路徑圖到: %s", self.save_path)
                except Exception as e:
                    rospy.logwarn("無法保存圖形: %s", str(e))
                
                plt.close()
        except Exception as e:
            rospy.logerr("繪圖更新錯誤: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        path_plotter = PathPlotter()
        path_plotter.run()
    except rospy.ROSInterruptException:
        pass 