#!/usr/bin/env python3
import rospy
import subprocess
import os
import time
import math
import cv2
import copy  # 導入copy模組，用於深拷貝
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge

class DroneFlyAndCapture:
    def __init__(self):
        rospy.init_node('drone_fly_and_capture_ekf', anonymous=True)

        # 影像存檔資料夾
        self.forward_save_path = "/home/jim/gazebo_images/forward/"
        self.return_save_path = "/home/jim/gazebo_images/return/"
        os.makedirs(self.forward_save_path, exist_ok=True)
        os.makedirs(self.return_save_path, exist_ok=True)

        # 影像轉換工具
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_vision/usb_cam/image_raw", Image, self.image_callback)

        # ROS Publisher 與 Service
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        # 訂閱EKF輸出的位姿和狀態
        rospy.Subscriber('/px4_style_ekf/pose', PoseWithCovarianceStamped, self.ekf_pose_callback)
        rospy.Subscriber('/ekf/status', State, self.ekf_status_callback)

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        self.current_state = State()
        self.current_position = PoseStamped()
        self.current_gps = None
        self.ekf_position = None
        self.ekf_status = None
        self.rate = rospy.Rate(20)  # 20Hz
        self.current_image = None
        self.home_ekf_position = None
        self.home_ekf_yaw = 0.0

        # 可調增益參數
        self.shift_kp = 0.5   # 位移補償增益
        self.angle_kp = 0.5   # <<< 恢復角度補償增益

        # 回程和控制參數
        self.goal_threshold = 2.0     # 到達目標距離閾值 (米)
        self.max_yaw_rate = 0.5       # 最大偏航角速度 (rad/s)
        self.return_speed = 2.0       # 回程速度 (m/s)
        self.max_lateral_speed = 0.5  # 最大側向速度 (m/s)
        self.photo_interval = 5.0     # 回程拍照間隔 (秒)
        self.last_photo_time = rospy.Time.now() # 上次拍照時間

        # 起點位置資訊
        self.start_position = None  # 儲存EKF起點位置
        self.current_ekf_yaw = 0.0  # 目前EKF估計的yaw角

        # 診斷用變量
        self.last_logged_position = None
        self.last_pos_log_time = None

        # 輸出訊息
        rospy.loginfo("無人機控制節點已初始化，等待其他系統...")

    # -------------------------- ROS Callback -----------------------------
    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_position = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"影像處理錯誤: {e}")

    # EKF位姿回調函數
    def ekf_pose_callback(self, msg):
        self.ekf_position = msg
        # 獲取最新的yaw角度估計
        self.current_ekf_yaw = self.get_ekf_yaw()
        
    # EKF狀態回調函數
    def ekf_status_callback(self, msg):
        self.ekf_status = msg
        
    # ------------------------- 輔助函式：取出目前 yaw -------------------------
    def get_current_yaw(self):
        q = self.current_position.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # 取得EKF估計的yaw角
    def get_ekf_yaw(self):
        if self.ekf_position is None:
            return 0.0
        q = self.ekf_position.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # --------------------------- 控制相關函式 -----------------------------
    def arm(self):
        rospy.loginfo("正在解鎖無人機 (Arming)...")
        for _ in range(5):
            if self.arm_service(True).success:
                rospy.loginfo("無人機解鎖成功！")
                return True
            rospy.sleep(1)
        rospy.logwarn("解鎖失敗。")
        return False

    def set_offboard_mode(self):
        rospy.loginfo("切換至 OFFBOARD 模式...")
        for _ in range(5):
            if self.set_mode_service(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("OFFBOARD 模式啟動成功！")
                return True
            rospy.sleep(1)
        rospy.logwarn("OFFBOARD 模式切換失敗。")
        return False

    def set_velocity(self, vx=0, vy=0, vz=0, yaw_rate=0):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = "base_link"
        
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        vel_msg.twist.angular.z = yaw_rate
        self.velocity_pub.publish(vel_msg)

    def takeoff(self, target_altitude=20):
        rospy.loginfo(f"起飛至 {target_altitude} m 高度...")
        if not self.arm():
            return
        if not self.set_offboard_mode():
            return
        
        # 起飛到指定高度
        while self.current_position.pose.position.z < target_altitude - 1 and not rospy.is_shutdown():
            self.set_velocity(vz=2.0)
            rospy.sleep(0.2)
        self.set_velocity(vz=0)
        rospy.loginfo(f"成功達到 {target_altitude} m！")
        
        # 在起點儲存EKF位置
        if self.ekf_position is not None:
            self.home_ekf_position = self.ekf_position.pose.pose.position
            self.home_ekf_yaw = self.get_ekf_yaw()
            rospy.loginfo(f"已儲存起點EKF位置 (NED): x={self.home_ekf_position.x:.2f}, y={self.home_ekf_position.y:.2f}, z={self.home_ekf_position.z:.2f}, yaw={math.degrees(self.home_ekf_yaw):.1f}°")
        else:
            rospy.logwarn("無法獲取EKF位置，無法儲存起點位置")
            return False

        # 拍攝起點影像供未來使用
        start_image_path = self.save_image(is_returning=False)
        if start_image_path is not None:
            rospy.loginfo(f"起點影像已儲存：{start_image_path}")

        return True

    def land(self):
        rospy.loginfo("開始降落...")
        self.set_mode_service(custom_mode="AUTO.LAND")
        rospy.loginfo("無人機正在降落。")

    def turn_around(self, target_angle_change=math.pi, yaw_rate_gain=0.8, tolerance=math.radians(2.0), timeout=30.0):
        """閉迴路控制旋轉指定角度"""
        rospy.loginfo(f"開始閉迴路轉向 {math.degrees(target_angle_change):.1f} 度...")

        if self.current_ekf_yaw is None:
            rospy.logwarn("無法獲取當前EKF Yaw，取消轉向")
            return False

        start_yaw = self.current_ekf_yaw
        target_yaw = self.normalize_angle(start_yaw + target_angle_change)

        rospy.loginfo(f"起始 Yaw (ENU): {math.degrees(self.normalize_angle(-start_yaw)):.1f}°, 目標 Yaw (ENU): {math.degrees(self.normalize_angle(-target_yaw)):.1f}°")

        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(20) # 控制頻率

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            if current_time - start_time > timeout:
                rospy.logwarn("轉向超時!")
                self.set_velocity(yaw_rate=0)
                return False

            if self.current_ekf_yaw is None:
                 rospy.logwarn_throttle(5, "轉向中等待EKF Yaw...")
                 rate.sleep()
                 continue

            current_yaw_ned = self.current_ekf_yaw
            yaw_error = self.normalize_angle(target_yaw - current_yaw_ned)

            # 顯示 ENU 角度方便觀察
            current_yaw_enu = self.normalize_angle(-current_yaw_ned)
            rospy.loginfo_throttle(1.0, f"轉向中... 當前 Yaw (ENU): {math.degrees(current_yaw_enu):.1f}°, 剩餘誤差: {math.degrees(yaw_error):.1f}°")

            if abs(yaw_error) < tolerance:
                rospy.loginfo("轉向完成，誤差已在容忍範圍內。")
                self.set_velocity(yaw_rate=0)
                return True

            # 根據誤差計算角速度 (P控制器)
            # 注意：yaw_rate 指令是針對機體Z軸的，與 NED/ENU 的 yaw 變化方向可能需要 +/- 號調整
            # 假設正的 yaw_rate 使 NED yaw 增加 (順時針)，使 ENU yaw 減少 (逆時針)
            # 我們希望 yaw_error 減小。如果 target_yaw > current_yaw_ned (誤差為正，需要順時針轉)，
            # 則需要正的 yaw_rate。
            # 如果 target_yaw < current_yaw_ned (誤差為負，需要逆時針轉)，
            # 則需要負的 yaw_rate。
            # 所以 yaw_rate = gain * yaw_error 應該是正確的。
            desired_yaw_rate = -yaw_rate_gain * yaw_error
            # 限制最大角速度
            desired_yaw_rate = max(min(desired_yaw_rate, self.max_yaw_rate), -self.max_yaw_rate)

            self.set_velocity(yaw_rate=desired_yaw_rate)
            rate.sleep()

        self.set_velocity(yaw_rate=0) # 如果 rospy is shutdown
        return False

    # --------------------------- 影像保存函式 -----------------------------
    def save_image(self, is_returning=False):
        if self.current_image is None:
            rospy.logwarn("無法儲存影像：current_image 為空")
            return None

        if is_returning:
            count = len([f for f in os.listdir(self.return_save_path) if f.endswith('.jpg')])
            filename = f"{count + 1}.jpg"
            save_path = os.path.join(self.return_save_path, filename)
        else:
            # 以當下GPS經緯度作為檔名（格式："23.451147,120.282450.jpg"），若GPS尚未取得則採編號方式
            if self.current_gps is not None:
                filename = f"{self.current_gps.latitude:.6f},{self.current_gps.longitude:.6f}.jpg"
            else:
                count = len([f for f in os.listdir(self.forward_save_path) if f.endswith('.jpg')])
                filename = f"{count + 1}.jpg"
            save_path = os.path.join(self.forward_save_path, filename)
        cv2.imwrite(save_path, self.current_image)
        rospy.loginfo(f"影像已儲存：{save_path}")
        return save_path

    # --------------------------- 飛行邏輯 (2D) -----------------------------
    def fly_straight_xy(self, start_x, start_y, end_x, end_y, speed, capture_interval):
        # capture_interval 以秒為單位
        init_x = self.current_position.pose.position.x
        init_y = self.current_position.pose.position.y

        dx = end_x - init_x
        dy = end_y - init_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 0.1:
            rospy.logwarn("目標距離太短，不執行飛行。")
            return

        heading = math.atan2(dy, dx)
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)

        traveled = 0.0
        last_capture_time = time.time()

        rospy.loginfo(f"開始飛行至目標 (x={end_x:.2f}, y={end_y:.2f}), 總距離: {distance:.2f} m, heading: {heading:.2f} rad")
        rate = rospy.Rate(20)
        while traveled < distance and not rospy.is_shutdown():
            current_yaw = self.get_current_yaw()
            yaw_error = self.normalize_angle(heading - current_yaw)
            desired_yaw_rate = self.angle_kp * yaw_error

            self.set_velocity(vx=vx, vy=vy, vz=0, yaw_rate=desired_yaw_rate)

            current_x = self.current_position.pose.position.x
            current_y = self.current_position.pose.position.y
            traveled = math.sqrt((current_x - init_x)**2 + (current_y - init_y)**2)

            current_time = time.time()
            if current_time - last_capture_time >= capture_interval:
                self.save_image(is_returning=False)
                last_capture_time = current_time

            rate.sleep()

        self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)
        rospy.loginfo("到達目標或已超過所需距離。")

    # --------------------------- 基於EKF的回程邏輯 -----------------------------
    def ekf_return_home(self):
        """基於EKF估計的位置返回起點"""
        rospy.loginfo("開始基於EKF的回家...")
        if self.home_ekf_position is None:
            rospy.logwarn("EKF 起點位置未設定，無法執行回家！")
            return

        rate = rospy.Rate(10) # 控制頻率 10Hz
        step = 0
        max_steps = 600 # 最多運行 60 秒

        while not rospy.is_shutdown() and step < max_steps:
            if not self.current_state.mode == "OFFBOARD" or not self.current_state.armed:
                rospy.logwarn("無人機不在OFFBOARD模式或未解鎖，停止回家。")
                break
                
            if self.ekf_position is None or self.current_ekf_yaw is None:
                 rospy.logwarn_throttle(5, "等待EKF位置和姿態數據...")
                 rate.sleep()
                 step += 1
                 continue

            rospy.loginfo("[Step {}] 開始基於EKF的回程...".format(step + 1)) # Keep this line
            rospy.loginfo("[Step {}] 目標起點 (NED): x={:.2f}, y={:.2f}, z={:.2f}".format(step + 1, self.home_ekf_position.x, self.home_ekf_position.y, self.home_ekf_position.z))

            # 獲取當前 EKF 位置 (NED)
            current_pos_ned = self.ekf_position.pose.pose.position
            rospy.loginfo("[Step {}] 當前EKF位置 (NED): x={:.2f}, y={:.2f}, z={:.2f}".format(step + 1, current_pos_ned.x, current_pos_ned.y, current_pos_ned.z))

            # 計算到起點的向量 (NED)
            dx_ned = self.home_ekf_position.x - current_pos_ned.x
            dy_ned = self.home_ekf_position.y - current_pos_ned.y
            # dz = self.home_ekf_position.z - current_pos_ned.z # Z軸通常由高度控制器處理

            # --- 座標系轉換 (NED -> ENU for heading calculation) ---
            # 注意：座標轉換要注意方向一致性！
            # NED：x指北，y指東，z指下
            # ENU：x指東，y指北，z指上
            # NED的x對應ENU的y，NED的y對應ENU的x
            dx_enu = dy_ned  # East difference (NED的y → ENU的x)
            dy_enu = dx_ned  # North difference (NED的x → ENU的y)
            
            # 重要：應該計算向量方向從當前位置指向起點，確保方向一致
            # dx_enu = dy_ned (NED的y軸 → ENU的x軸)
            # dy_enu = dx_ned (NED的x軸 → ENU的y軸)
            
            distance_to_home = math.sqrt(dx_enu**2 + dy_enu**2)
            rospy.loginfo("[Step {}] 到起點距離: {:.2f} m, dN={:.2f}, dE={:.2f}".format(step + 1, distance_to_home, dx_ned, dy_ned)) # Log NED differences
            # 增加更詳細的向量信息
            rospy.loginfo("[Step {}] 向量診斷: NED向量=({:.2f},{:.2f}), ENU向量=({:.2f},{:.2f})".format(step + 1, dx_ned, dy_ned, dx_enu, dy_enu))
            
            # 計算朝向起點的目標航向角 (ENU, 逆時針 from North)
            # atan2(East, North) -> 從北方開始逆時針測量的角度
            target_heading_enu = math.atan2(dx_enu, dy_enu)
            rospy.loginfo("[Step {}] 目標航向計算: atan2({:.2f}, {:.2f}) = {:.1f}°".format(step + 1, dx_enu, dy_enu, math.degrees(target_heading_enu)))

            # 獲取當前 EKF 航向角 (NED, 順時針 from North)
            current_yaw_ned = self.current_ekf_yaw # <<< 恢復使用 EKF Yaw

            # 轉換當前航向角到 ENU (逆時針 from North)
            current_yaw_enu = self.normalize_angle(-current_yaw_ned)
            # --- 座標系轉換結束 ---

            # 計算航向誤差 (ENU)
            heading_error = self.normalize_angle(target_heading_enu - current_yaw_enu)

            # 在日誌中明確標示使用的是 ENU 航向
            rospy.loginfo(f"[Step {step+1}] 當前航向 (ENU): {math.degrees(current_yaw_enu):.1f}°, 目標航向 (ENU): {math.degrees(target_heading_enu):.1f}°, 誤差: {math.degrees(heading_error):.1f}°")


            # 如果接近起點，則嘗試降落
            if distance_to_home < self.goal_threshold:
                rospy.loginfo("已接近EKF起點，準備降落。")
                self.land()
                break

            # --- 計算速度指令 (基於 ENU 航向誤差) ---
            # 根據航向誤差計算角速度指令 (yaw_rate)
            yaw_rate = self.angle_kp * heading_error  # <<< 恢復為正增益，根據標準約定和 turn_around 的結果，這應該是正確的
            # 限制最大角速度
            yaw_rate = max(min(yaw_rate, self.max_yaw_rate), -self.max_yaw_rate)

            # 根據航向誤差計算前進和側向速度 (FLU - 機體座標系)
            # 如果誤差大，主要轉向；誤差小，則前進
            if abs(math.degrees(heading_error)) > 15: # 如果航向誤差大於15度
                # forward_speed = 0.2 # 減慢速度專心轉向
                forward_speed = 0.0 # <<< 修改：大角度誤差時，原地旋轉，不前進
                lateral_speed = 0.0 # <<< 保持：不需要側向移動
                # rospy.loginfo(f"[Step {step+1}] 大角度轉向中...")
                rospy.loginfo(f"[Step {step+1}] 大角度原地轉向中...") # <<< 修改日誌訊息
            else:
                # 誤差小時，主要前進
                # 【修改】：不再使用固定前進速度，而是基於位置誤差計算速度向量
                rospy.loginfo(f"[Step {step+1}] 小角度修正並前進...")
                
                # 計算位置誤差向量 (NED)
                position_error_n = self.home_ekf_position.x - current_pos_ned.x  # 北向誤差
                position_error_e = self.home_ekf_position.y - current_pos_ned.y  # 東向誤差
                
                # 計算在世界座標系下的期望速度向量 (NED)
                desired_vel_n = position_error_n / distance_to_home * self.return_speed  # 北向速度
                desired_vel_e = position_error_e / distance_to_home * self.return_speed  # 東向速度
                
                # 轉換到機體座標系 (FLU)
                # 注意：current_yaw_ned 是機體朝向在 NED 系統中的角度
                # 我們需要把 NED 中的速度轉到機體座標系下
                cos_yaw = math.cos(current_yaw_ned)
                sin_yaw = math.sin(current_yaw_ned)
                
                # 在機體座標系下的期望速度
                # 前向速度 = 北向分量在機頭方向的投影 + 東向分量在機頭方向的投影
                forward_speed = cos_yaw * desired_vel_n + sin_yaw * desired_vel_e
                # 左向速度 = 北向分量在機左方向的投影 + 東向分量在機左方向的投影
                lateral_speed = -sin_yaw * desired_vel_n + cos_yaw * desired_vel_e
                
                # 限制最大速度
                if abs(forward_speed) > self.return_speed:
                    forward_speed = math.copysign(self.return_speed, forward_speed)
                if abs(lateral_speed) > self.max_lateral_speed:
                    lateral_speed = math.copysign(self.max_lateral_speed, lateral_speed)
                
                # 記錄診斷信息
                rospy.loginfo(f"[Step {step+1}] 前進診斷: 向起點方向={math.degrees(target_heading_enu):.1f}°, 當前航向={math.degrees(current_yaw_enu):.1f}°")
                rospy.loginfo(f"[Step {step+1}] 位置誤差: dN={position_error_n:.2f}, dE={position_error_e:.2f} (NED)")
                rospy.loginfo(f"[Step {step+1}] 期望速度: Vn={desired_vel_n:.2f}, Ve={desired_vel_e:.2f} (NED)")
                rospy.loginfo(f"[Step {step+1}] 機體速度: 前向={forward_speed:.2f}, 側向={lateral_speed:.2f} (FLU)")
                # 以下診斷信息保留
                expected_dx = math.sin(current_yaw_enu) * forward_speed  # ENU: sin(yaw) 對應東向分量
                expected_dy = math.cos(current_yaw_enu) * forward_speed  # ENU: cos(yaw) 對應北向分量
                rospy.loginfo(f"[Step {step+1}] 預期移動: dx={expected_dx:.2f}, dy={expected_dy:.2f} (ENU)")
            # --- 速度指令計算結束 ---


            # 發布速度指令 (FLU座標系: Forward-Left-Up)
            vel_msg = TwistStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.header.frame_id = "base_link"

            vel_msg.twist.linear.x = forward_speed
            vel_msg.twist.linear.y = lateral_speed # 如果上面用了 sin(error) 方式，這裡才設值
            vel_msg.twist.linear.z = 0 # 高度由 takeoff_and_hover 控制或獨立的Z控制器處理
            vel_msg.twist.angular.z = yaw_rate

            self.velocity_pub.publish(vel_msg)
            rospy.loginfo(f"[Step {step+1}] 發布指令: Fwd={vel_msg.twist.linear.x:.2f}, Lat={vel_msg.twist.linear.y:.2f}, YawRate={vel_msg.twist.angular.z:.2f}")


            # 定期拍照
            current_time = rospy.Time.now()
            if (current_time - self.last_photo_time).to_sec() >= self.photo_interval:
                self.save_image(is_returning=True)
                self.last_photo_time = current_time

            # 記錄實際運動診斷信息
            if self.last_pos_log_time is None or (current_time - self.last_pos_log_time).to_sec() >= 1.0:
                if self.last_logged_position is not None:
                    # 計算實際移動（NED坐標系）
                    actual_dn = current_pos_ned.x - self.last_logged_position.x
                    actual_de = current_pos_ned.y - self.last_logged_position.y
                    # 轉換為ENU坐標系
                    actual_dx = actual_de  # NED的y對應ENU的x
                    actual_dy = actual_dn  # NED的x對應ENU的y
                    actual_distance = math.sqrt(actual_dx**2 + actual_dy**2)
                    actual_direction = math.degrees(math.atan2(actual_dx, actual_dy))
                    actual_speed = actual_distance / 1.0  # 假設間隔1秒
                    rospy.loginfo(f"[實際運動] dx={actual_dx:.2f}, dy={actual_dy:.2f}, 方向={actual_direction:.1f}°, 速度={actual_speed:.2f}m/s")
                    
                    # 計算距離變化
                    prev_distance = math.sqrt((self.last_logged_position.x - self.home_ekf_position.x)**2 + 
                                             (self.last_logged_position.y - self.home_ekf_position.y)**2)
                    distance_change = distance_to_home - prev_distance
                    rospy.loginfo(f"[距離變化] 當前={distance_to_home:.2f}m, 之前={prev_distance:.2f}m, 變化={distance_change:.2f}m")
                
                # 使用深拷貝進行位置記錄
                self.last_logged_position = copy.deepcopy(current_pos_ned)
                self.last_pos_log_time = current_time

            rate.sleep()
            step += 1

        if step >= max_steps:
             rospy.logwarn("回家超時！")
             self.land() # 超時也嘗試降落

    def fly_and_capture(self, start_x, start_y, end_x, end_y, speed, capture_interval):
        rospy.loginfo("準備起飛...")
        self.takeoff(target_altitude=20)

        rospy.loginfo("開始往前 (XY) 飛行並拍照...")
        self.fly_straight_xy(start_x, start_y, end_x, end_y, speed, capture_interval)

        # 拍攝轉向前的參考影像
        rospy.loginfo("轉向前拍攝參考影像...")
        reference_image_path = self.save_image(is_returning=False)
        if reference_image_path is not None:
            rospy.loginfo(f"轉向前參考影像已儲存：{reference_image_path}")

        # self.turn_around() # <<< 註釋掉此行，讓 ekf_return_home 處理回程轉向

        rospy.loginfo("開始基於EKF的回家...")
        if self.ekf_return_home(): # 現在 ekf_return_home 會處理初始的大角度轉向
            rospy.loginfo("成功返回起點，準備降落...")
        else:
            rospy.logwarn("無法返回起點，嘗試降落在當前位置...")
            
        self.land()
        rospy.loginfo("任務完成.")

if __name__ == '__main__':
    try:
        drone = DroneFlyAndCapture()
        while not drone.current_state.connected:
            rospy.loginfo("等待 FCU 連線...")
            rospy.sleep(1)
        # 設置 capture_interval 參數為10秒
        drone.fly_and_capture(
            start_x=0,
            start_y=0,
            end_x=30,
            end_y=30,
            speed=5.0,
            capture_interval=10
        )
    except rospy.ROSInterruptException:
        rospy.loginfo("程式被中斷。") 