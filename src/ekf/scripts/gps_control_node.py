#!/usr/bin/env python3
import rospy
import threading
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt32, Bool
from std_srvs.srv import SetBool, SetBoolResponse

class GPSControlNode:
    """
    GPS控制節點，用於動態啟用/禁用GPS資料流向EKF
    提供服務來控制GPS資料傳輸，以模擬GPS丟失的情況
    """
    def __init__(self):
        rospy.init_node('gps_control_node')
        
        # 控制標誌，預設為啟用GPS
        self.gps_enabled = True
        self.lock = threading.RLock()
        
        # 讀取參數
        self.mavros_gps_topic = rospy.get_param('~mavros_gps_topic', '/mavros/global_position/raw/fix')
        self.mavros_gps_vel_topic = rospy.get_param('~mavros_gps_vel_topic', '/mavros/global_position/raw/gps_vel')
        self.mavros_gps_sat_topic = rospy.get_param('~mavros_gps_sat_topic', '/mavros/global_position/raw/satellites')
        
        self.ekf_gps_topic = rospy.get_param('~ekf_gps_topic', '/gps_data')
        self.ekf_gps_vel_topic = rospy.get_param('~ekf_gps_vel_topic', '/gps_vel_data')
        self.ekf_gps_sat_topic = rospy.get_param('~ekf_gps_sat_topic', '/gps_sat_data')
        
        # 服務：啟用/禁用GPS
        self.srv_enable_gps = rospy.Service('~enable_gps', SetBool, self.handle_enable_gps)
        
        # 發布GPS使能狀態
        self.pub_gps_status = rospy.Publisher('~status', Bool, queue_size=1)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # 訂閱原始GPS數據
        self.sub_gps = rospy.Subscriber(self.mavros_gps_topic, NavSatFix, self.gps_callback)
        self.sub_gps_vel = rospy.Subscriber(self.mavros_gps_vel_topic, TwistStamped, self.gps_vel_callback)
        self.sub_gps_sat = rospy.Subscriber(self.mavros_gps_sat_topic, UInt32, self.gps_sat_callback)
        
        # 發布過濾後的GPS數據
        self.pub_gps = rospy.Publisher(self.ekf_gps_topic, NavSatFix, queue_size=1)
        self.pub_gps_vel = rospy.Publisher(self.ekf_gps_vel_topic, TwistStamped, queue_size=1)
        self.pub_gps_sat = rospy.Publisher(self.ekf_gps_sat_topic, UInt32, queue_size=1)
        
        # 無效GPS數據 (當禁用GPS時使用)
        self.invalid_gps = NavSatFix()
        self.invalid_gps.status.status = -1  # 無效狀態
        self.invalid_gps.status.service = 0  # 無服務
        self.invalid_gps.latitude = 0.0
        self.invalid_gps.longitude = 0.0
        self.invalid_gps.altitude = 0.0
        self.invalid_gps.position_covariance_type = 0  # 未知協方差
        
        # 無效GPS速度
        self.invalid_gps_vel = TwistStamped()
        
        # 無效GPS衛星數量 (設為0)
        self.invalid_gps_sat = UInt32(0)
        
        rospy.loginfo("GPS控制節點已初始化。使用服務'/gps_control_node/enable_gps'來控制GPS")
        
        # 添加更明確的準備就緒訊息
        rospy.loginfo("GPS控制服務 '/gps_control_node/enable_gps' 已啟動並可用")
        rospy.loginfo("--------------------------------------------------------------------")
    
    def handle_enable_gps(self, req):
        """處理啟用/禁用GPS的服務請求"""
        with self.lock:
            self.gps_enabled = req.data
            status = "啟用" if self.gps_enabled else "禁用"
            rospy.loginfo(f"GPS已{status}")
            
            # 如果禁用GPS，立即發布無效GPS數據
            if not self.gps_enabled:
                self.invalid_gps.header.stamp = rospy.Time.now()
                self.pub_gps.publish(self.invalid_gps)
                
                self.invalid_gps_vel.header.stamp = rospy.Time.now()
                self.pub_gps_vel.publish(self.invalid_gps_vel)
                
                self.pub_gps_sat.publish(self.invalid_gps_sat)
                
            return SetBoolResponse(success=True, message=f"GPS已{status}")
    
    def publish_status(self, event):
        """定期發布GPS使能狀態"""
        with self.lock:
            self.pub_gps_status.publish(Bool(self.gps_enabled))
    
    def gps_callback(self, msg):
        """處理GPS數據回調"""
        with self.lock:
            if self.gps_enabled:
                self.pub_gps.publish(msg)
            else:
                # 如果GPS被禁用，發布無效數據
                self.invalid_gps.header.stamp = rospy.Time.now()
                self.pub_gps.publish(self.invalid_gps)
    
    def gps_vel_callback(self, msg):
        """處理GPS速度數據回調"""
        with self.lock:
            if self.gps_enabled:
                self.pub_gps_vel.publish(msg)
            else:
                # 如果GPS被禁用，發布無效速度數據
                self.invalid_gps_vel.header.stamp = rospy.Time.now()
                self.pub_gps_vel.publish(self.invalid_gps_vel)
    
    def gps_sat_callback(self, msg):
        """處理GPS衛星數量數據回調"""
        with self.lock:
            if self.gps_enabled:
                self.pub_gps_sat.publish(msg)
            else:
                # 如果GPS被禁用，發布0衛星
                self.pub_gps_sat.publish(self.invalid_gps_sat)

if __name__ == '__main__':
    try:
        node = GPSControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 