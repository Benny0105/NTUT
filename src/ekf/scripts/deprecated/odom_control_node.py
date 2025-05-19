#!/usr/bin/env python3
import rospy
import threading
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
import tf.transformations as tf_trans

class OdometryControlNode:
    """
    視覺里程計控制節點，用於動態啟用/禁用里程計資料流向EKF
    提供服務來控制里程計資料傳輸，以模擬視覺里程計失效或漂移的情況
    """
    def __init__(self):
        rospy.init_node('odom_control_node')
        
        # 控制標誌，預設為啟用里程計
        self.odom_enabled = True
        self.lock = threading.RLock()
        
        # 讀取參數
        self.mavros_odom_topic = rospy.get_param('~mavros_odom_topic', '/mavros/odometry/out')
        self.mavros_vision_pose_topic = rospy.get_param('~mavros_vision_pose_topic', '/mavros/vision_pose/pose')
        
        self.ekf_odom_topic = rospy.get_param('~ekf_odom_topic', '/odom_data')
        
        # 服務：啟用/禁用里程計
        self.srv_enable_odom = rospy.Service('~enable_odom', SetBool, self.handle_enable_odom)
        
        # 發布里程計使能狀態
        self.pub_odom_status = rospy.Publisher('~status', Bool, queue_size=1)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # 訂閱原始里程計數據
        self.sub_odom = rospy.Subscriber(self.mavros_odom_topic, Odometry, self.odom_callback)
        self.sub_vision_pose = rospy.Subscriber(self.mavros_vision_pose_topic, PoseWithCovarianceStamped, self.vision_pose_callback)
        
        # 發布過濾後的里程計數據
        self.pub_odom = rospy.Publisher(self.ekf_odom_topic, Odometry, queue_size=1)
        
        # 無效里程計數據 (當禁用里程計時使用)
        self.invalid_odom = Odometry()
        self.invalid_odom.header.frame_id = "odom"
        self.invalid_odom.child_frame_id = "base_link"
        # 設置無效位置協方差
        self.invalid_odom.pose.covariance = [9999.0] * 36
        # 設置無效速度協方差
        self.invalid_odom.twist.covariance = [9999.0] * 36
        
        # 里程計計算參數
        self.last_vision_pose = None
        self.last_vision_time = None
        self.vision_valid = False
        
        # 位置和速度過濾器參數
        self.position_filter_alpha = 0.2  # 位置低通濾波參數
        self.velocity_filter_alpha = 0.3  # 速度低通濾波參數
        
        # 漂移檢測參數
        self.max_pos_jump = 1.0  # 最大位置跳變 (米)
        self.max_vel = 10.0      # 最大合理速度 (米/秒)
        
        # 濾波後的里程計數據
        self.filtered_odom = Odometry()
        self.filtered_odom.header.frame_id = "odom"
        self.filtered_odom.child_frame_id = "base_link"
        
        rospy.loginfo("視覺里程計控制節點已初始化。使用服務'/odom_control_node/enable_odom'來控制視覺里程計")
        
        # 添加更明確的準備就緒訊息
        rospy.loginfo("里程計控制服務 '/odom_control_node/enable_odom' 已啟動並可用")
        rospy.loginfo("--------------------------------------------------------------------")
    
    def handle_enable_odom(self, req):
        """處理啟用/禁用里程計的服務請求"""
        with self.lock:
            self.odom_enabled = req.data
            status = "啟用" if self.odom_enabled else "禁用"
            rospy.loginfo(f"視覺里程計已{status}")
            
            # 如果禁用里程計，立即發布無效里程計數據
            if not self.odom_enabled:
                self.invalid_odom.header.stamp = rospy.Time.now()
                self.pub_odom.publish(self.invalid_odom)
                
            return SetBoolResponse(success=True, message=f"視覺里程計已{status}")
    
    def publish_status(self, event):
        """定期發布里程計使能狀態"""
        with self.lock:
            self.pub_odom_status.publish(Bool(self.odom_enabled))
    
    def odom_callback(self, msg):
        """處理MAVROS里程計數據回調"""
        with self.lock:
            if not self.odom_enabled:
                # 如果里程計被禁用，發布無效數據
                self.invalid_odom.header.stamp = rospy.Time.now()
                self.pub_odom.publish(self.invalid_odom)
                return
                
            # 檢查數據有效性
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            linear = msg.twist.twist.linear
            
            # 基本的有效性檢查
            if (np.isnan([position.x, position.y, position.z, 
                         orientation.x, orientation.y, orientation.z, orientation.w,
                         linear.x, linear.y, linear.z]).any() or
                np.isinf([position.x, position.y, position.z,
                         orientation.x, orientation.y, orientation.z, orientation.w,
                         linear.x, linear.y, linear.z]).any()):
                rospy.logwarn_throttle(5.0, "收到無效的MAVROS里程計數據，跳過")
                return
            
            # 過濾異常值
            vel_norm = np.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
            if vel_norm > self.max_vel:
                rospy.logwarn_throttle(1.0, f"檢測到異常速度: {vel_norm:.2f} m/s > {self.max_vel:.2f} m/s，降低權重")
                # 降低異常速度的權重
                self.velocity_filter_alpha = 0.05
            else:
                # 正常速度使用標準權重
                self.velocity_filter_alpha = 0.3
            
            # 如果是第一次收到有效數據，直接使用
            if not hasattr(self, 'filtered_position'):
                self.filtered_position = np.array([position.x, position.y, position.z])
                self.filtered_velocity = np.array([linear.x, linear.y, linear.z])
                self.filtered_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            else:
                # 否則進行低通濾波
                current_position = np.array([position.x, position.y, position.z])
                current_velocity = np.array([linear.x, linear.y, linear.z])
                current_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
                
                # 檢測位置突變
                pos_jump = np.linalg.norm(current_position - self.filtered_position)
                if pos_jump > self.max_pos_jump:
                    rospy.logwarn_throttle(1.0, f"檢測到位置突變: {pos_jump:.2f} m > {self.max_pos_jump:.2f} m，降低位置更新權重")
                    pos_alpha = 0.05  # 突變時使用較小的權重
                else:
                    pos_alpha = self.position_filter_alpha
                
                # 應用低通濾波
                self.filtered_position = (1 - pos_alpha) * self.filtered_position + pos_alpha * current_position
                self.filtered_velocity = (1 - self.velocity_filter_alpha) * self.filtered_velocity + self.velocity_filter_alpha * current_velocity
                
                # 四元數需要特殊處理 - 使用球面線性插值(SLERP)
                q1 = [self.filtered_orientation[3], self.filtered_orientation[0], 
                      self.filtered_orientation[1], self.filtered_orientation[2]]  # w,x,y,z格式
                q2 = [current_orientation[3], current_orientation[0], 
                      current_orientation[1], current_orientation[2]]  # w,x,y,z格式
                
                q_slerp = tf_trans.quaternion_slerp(q1, q2, self.position_filter_alpha)
                self.filtered_orientation = np.array([q_slerp[1], q_slerp[2], q_slerp[3], q_slerp[0]])  # x,y,z,w格式
            
            # 更新濾波後的里程計信息
            self.filtered_odom.header.stamp = msg.header.stamp
            
            # 設置位置
            self.filtered_odom.pose.pose.position.x = self.filtered_position[0]
            self.filtered_odom.pose.pose.position.y = self.filtered_position[1]
            self.filtered_odom.pose.pose.position.z = self.filtered_position[2]
            
            # 設置姿態
            self.filtered_odom.pose.pose.orientation.x = self.filtered_orientation[0]
            self.filtered_odom.pose.pose.orientation.y = self.filtered_orientation[1]
            self.filtered_odom.pose.pose.orientation.z = self.filtered_orientation[2]
            self.filtered_odom.pose.pose.orientation.w = self.filtered_orientation[3]
            
            # 設置線速度
            self.filtered_odom.twist.twist.linear.x = self.filtered_velocity[0]
            self.filtered_odom.twist.twist.linear.y = self.filtered_velocity[1]
            self.filtered_odom.twist.twist.linear.z = self.filtered_velocity[2]
            
            # 設置角速度 (直接從原始消息中獲取)
            self.filtered_odom.twist.twist.angular = msg.twist.twist.angular
            
            # 設置協方差 (簡化起見，使用原始消息的協方差)
            self.filtered_odom.pose.covariance = msg.pose.covariance
            self.filtered_odom.twist.covariance = msg.twist.covariance
            
            # 發布濾波後的里程計
            self.pub_odom.publish(self.filtered_odom)
    
    def vision_pose_callback(self, msg):
        """處理MAVROS視覺位姿數據回調"""
        with self.lock:
            if not self.odom_enabled:
                return
                
            # 檢查數據有效性
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # 基本的有效性檢查
            if (np.isnan([position.x, position.y, position.z, 
                         orientation.x, orientation.y, orientation.z, orientation.w]).any() or
                np.isinf([position.x, position.y, position.z,
                         orientation.x, orientation.y, orientation.z, orientation.w]).any()):
                rospy.logwarn_throttle(5.0, "收到無效的視覺位姿數據，跳過")
                return
            
            current_time = msg.header.stamp
            current_position = np.array([position.x, position.y, position.z])
            
            # 如果是首次收到有效數據，僅記錄不進行速度計算
            if self.last_vision_pose is None or self.last_vision_time is None:
                self.last_vision_pose = current_position
                self.last_vision_time = current_time
                self.vision_valid = True
                return
            
            # 計算時間差
            dt = (current_time - self.last_vision_time).to_sec()
            if dt <= 0.001:  # 避免除以太小的數
                return
            
            # 計算速度
            velocity = (current_position - self.last_vision_pose) / dt
            
            # 檢查速度是否合理
            vel_norm = np.linalg.norm(velocity)
            if vel_norm > self.max_vel:
                rospy.logwarn_throttle(1.0, f"視覺位姿檢測到異常速度: {vel_norm:.2f} m/s，跳過更新")
                return
            
            # 如果沒有現有的里程計數據，使用視覺位姿數據創建
            if not hasattr(self, 'filtered_position'):
                self.filtered_position = current_position
                self.filtered_velocity = velocity
                self.filtered_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
                
                # 初始化里程計消息
                self.filtered_odom.header.stamp = current_time
                self.filtered_odom.pose.pose.position.x = current_position[0]
                self.filtered_odom.pose.pose.position.y = current_position[1]
                self.filtered_odom.pose.pose.position.z = current_position[2]
                self.filtered_odom.pose.pose.orientation = orientation
                self.filtered_odom.twist.twist.linear.x = velocity[0]
                self.filtered_odom.twist.twist.linear.y = velocity[1]
                self.filtered_odom.twist.twist.linear.z = velocity[2]
                
                # 發布里程計
                self.pub_odom.publish(self.filtered_odom)
            
            # 更新上次視覺數據
            self.last_vision_pose = current_position
            self.last_vision_time = current_time

if __name__ == '__main__':
    try:
        node = OdometryControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 