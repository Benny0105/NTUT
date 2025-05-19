#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from std_msgs.msg import Header

class PX4Control:
    def __init__(self):
        # 初始化ROS節點
        rospy.init_node('simple_px4_control', anonymous=True)
        self.rate = rospy.Rate(20)  # 20Hz
        
        # 狀態變量
        self.current_state = State()
        self.last_state_callback_time = 0
        
        # 訂閱MAVROS主題
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        
        # 創建發布者
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # 等待與飛控連接
        rospy.loginfo("等待飛控連接...")
        connected = self.wait_for_connection(60)  # 最多等待60秒
        if not connected:
            rospy.logerr("無法與飛控建立連接，退出...")
            return
        
        # 發送初始位置（起飛點）
        self.send_initial_setpoints()
        
        # 嘗試進入OFFBOARD模式
        if not self.set_offboard_mode():
            rospy.logerr("無法進入OFFBOARD模式，退出...")
            return
        
        # 嘗試解鎖無人機
        if not self.arm_vehicle():
            rospy.logerr("無法解鎖無人機，退出...")
            return
        
        # 執行簡單任務 - 上升到2米高度
        self.simple_mission()
        
        # 降落
        self.land()
    
    def state_callback(self, state_msg):
        """處理MAVROS狀態消息"""
        self.current_state = state_msg
        self.last_state_callback_time = rospy.Time.now().to_sec()
        
        # 定期顯示狀態信息
        rospy.loginfo_throttle(2.0, 
                             f"模式: {self.current_state.mode}, "
                             f"解鎖狀態: {'已解鎖' if self.current_state.armed else '未解鎖'}, "
                             f"連接: {'已連接' if self.current_state.connected else '未連接'}")
    
    def wait_for_connection(self, timeout):
        """等待與飛控建立連接"""
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.connected:
                rospy.loginfo("已連接到飛控!")
                return True
            
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                return False
            
            if int(elapsed) % 5 == 0:
                rospy.loginfo_throttle(5.0, f"等待飛控連接... 已等待 {int(elapsed)} 秒")
            
            self.rate.sleep()
        
        return False
    
    def send_initial_setpoints(self):
        """發送初始位置指令"""
        rospy.loginfo("發送初始位置指令...")
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        
        # 發送100次初始位置指令
        for i in range(100):
            if i % 25 == 0:
                rospy.loginfo(f"初始位置指令: {i+1}/100")
            
            pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
    
    def set_mode(self, mode):
        """設置飛行模式"""
        rospy.wait_for_service('mavros/set_mode')
        try:
            set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
            resp = set_mode_srv(0, mode)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"設置模式服務調用失敗: {e}")
            return False
    
    def arm(self, arm_cmd):
        """解鎖/上鎖無人機"""
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            resp = arm_srv(arm_cmd)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"解鎖服務調用失敗: {e}")
            return False
    
    def set_offboard_mode(self):
        """設置為OFFBOARD模式"""
        # 如果已經是OFFBOARD模式，先切換到其他模式
        if self.current_state.mode == "OFFBOARD":
            rospy.loginfo("當前已是OFFBOARD模式，嘗試切換到POSCTL")
            self.set_mode("POSCTL")
            rospy.sleep(1.0)
        
        # 嘗試設置OFFBOARD模式5次
        for attempt in range(5):
            rospy.loginfo(f"嘗試進入OFFBOARD模式 ({attempt+1}/5)...")
            if self.set_mode("OFFBOARD"):
                rospy.loginfo("成功進入OFFBOARD模式!")
                return True
            
            # 每次嘗試間發送更多的位置指令
            self.send_postion_setpoint(0, 0, 0, 20)
            rospy.sleep(1.0)
        
        return False
    
    def arm_vehicle(self):
        """解鎖無人機"""
        # 如果已經解鎖，先上鎖
        if self.current_state.armed:
            rospy.loginfo("無人機已解鎖，嘗試重新上鎖")
            self.arm(False)
            rospy.sleep(1.0)
        
        # 嘗試解鎖5次
        for attempt in range(5):
            rospy.loginfo(f"嘗試解鎖無人機 ({attempt+1}/5)...")
            
            # 確保在OFFBOARD模式
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
                rospy.sleep(0.5)
            
            # 發送一些位置指令再嘗試解鎖
            self.send_postion_setpoint(0, 0, 0, 10)
            
            if self.arm(True):
                rospy.loginfo("無人機解鎖成功!")
                return True
            
            rospy.sleep(1.0)
        
        return False
    
    def send_postion_setpoint(self, x, y, z, count=1):
        """發送位置指令"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        for i in range(count):
            pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(pose)
            self.rate.sleep()
    
    def simple_mission(self):
        """執行簡單任務 - 上升到2米高度"""
        rospy.loginfo("開始執行簡單任務 - 上升到2米高度")
        
        # 逐步上升到2米高度
        height_steps = [0.5, 1.0, 1.5, 2.0]
        for height in height_steps:
            rospy.loginfo(f"上升到 {height} 米")
            
            # 在每個高度停留3秒
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration(3.0):
                self.send_postion_setpoint(0, 0, height)
        
        # 在2米高度停留5秒
        rospy.loginfo("在2米高度懸停5秒")
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(5.0):
            self.send_postion_setpoint(0, 0, 2.0)
    
    def land(self):
        """執行降落程序"""
        rospy.loginfo("執行降落程序")
        
        # 嘗試切換到降落模式
        if self.set_mode("AUTO.LAND"):
            rospy.loginfo("已切換到降落模式")
        else:
            rospy.logwarn("無法切換到降落模式，嘗試手動降落")
            
            # 手動降落 - 逐步降低高度
            heights = [1.5, 1.0, 0.5, 0.0]
            for height in heights:
                rospy.loginfo(f"降低到 {height} 米")
                
                # 在每個高度停留2秒
                start_time = rospy.Time.now()
                while rospy.Time.now() - start_time < rospy.Duration(2.0):
                    self.send_postion_setpoint(0, 0, height)
        
        # 完成降落後上鎖
        rospy.sleep(2.0)
        if self.arm(False):
            rospy.loginfo("無人機已上鎖")
        else:
            rospy.logwarn("無法上鎖無人機")

if __name__ == '__main__':
    try:
        controller = PX4Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 