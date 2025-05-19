#!/usr/bin/env python3
import rospy
import subprocess
import os
import time
import math
import cv2
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge

# 匯入 feature 模組的各函式（注意新增 find_most_similar_images）
from feature2 import (compute_shift_and_angle, compute_shift_and_angle_single,
                     compute_target_offset_from_match, compute_top_center_offset, 
                     save_matching_centers_annotation, compute_dx_dy_between_images,
                     find_most_similar_images)

class DroneFlyAndCapture:
    def __init__(self):
        rospy.init_node('drone_fly_and_capture_xy', anonymous=True)

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

        # 訂閱EKF輸出的位姿和狀態（新增）
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
        self.ekf_status = None  # 新增：存儲EKF狀態
        self.rate = rospy.Rate(20)  # 20Hz
        self.current_image = None

        # 可調增益參數
        self.shift_kp = 0.5   # 位移補償增益
        self.angle_kp = 0.5   # 角度補償增益

        # 用來儲存起點影像的檔名與GPS
        self.start_image_filename = None
        self.start_gps = None

        # 輸出訊息
        rospy.loginfo("無人機控制節點已初始化，等待其他系統...")

    # -------------------------- ROS Callback -----------------------------
    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_position = msg

    def gps_callback(self, msg):
        # 移除 GPS 干擾，直接儲存接收到的 GPS 訊息
        self.current_gps = msg

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"影像處理錯誤: {e}")

    # 新增：EKF位姿回調函數
    def ekf_pose_callback(self, msg):
        self.ekf_position = msg
        
    # 新增：EKF狀態回調函數
    def ekf_status_callback(self, msg):
        self.ekf_status = msg
        
    # ------------------------- 輔助函式：取出目前 yaw -------------------------
    def get_current_yaw(self):
        q = self.current_position.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # 新增：取得EKF估計的yaw角
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

    # --------------------------- 新增：插值函式 -----------------------------
    def print_interpolated_gps(self, lat1, lon1, lat2, lon2):
        """
        線性插值產生包含兩端點的10個GPS點，並印出在終端機
        """
        rospy.loginfo("插值經緯度:")
        for i in range(10):
            fraction = i / 9.0  # fraction從0到1，包含兩端點
            interp_lat = lat1 + fraction * (lat2 - lat1)
            interp_lon = lon1 + fraction * (lon2 - lon1)
            rospy.loginfo(f"點 {i+1}: ({interp_lat:.6f}, {interp_lon:.6f})")

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
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        vel_msg.twist.angular.z = yaw_rate
        vel_msg.header.stamp = rospy.Time.now()
        self.velocity_pub.publish(vel_msg)

    def takeoff(self, target_altitude=50):
        rospy.loginfo(f"起飛至 {target_altitude} m 高度...")
        if not self.arm():
            return
        if not self.set_offboard_mode():
            return
        while self.current_position.pose.position.z < target_altitude - 1 and not rospy.is_shutdown():
            self.set_velocity(vz=20.0)
            rospy.sleep(0.2)
        self.set_velocity(vz=0)
        rospy.loginfo(f"成功達到 {target_altitude} m！")
        # 拍攝起點影像，作為返程比對的起始參考影像
        start_image_path = self.save_image(is_returning=False)
        if start_image_path is not None:
            self.start_image_filename = os.path.basename(start_image_path)
            # 解析檔名取得起點GPS
            parts = os.path.splitext(self.start_image_filename)[0].split(',')
            if len(parts) == 2:
                try:
                    self.start_gps = (float(parts[0]), float(parts[1]))
                    rospy.loginfo(f"起點GPS: {self.start_gps}")
                except ValueError:
                    rospy.logwarn("起點影像檔名格式錯誤，無法解析GPS")
                    self.start_gps = None
            else:
                rospy.logwarn("起點影像檔名格式錯誤")
            rospy.loginfo(f"起點影像已儲存：{start_image_path}")

    def land(self):
        rospy.loginfo("開始降落...")
        self.set_mode_service(custom_mode="AUTO.LAND")
        rospy.loginfo("無人機正在降落。")

    def turn_around(self, yaw_rate=0.5, angle=math.pi):
        rospy.loginfo("開始轉向...")
        turn_duration = angle / abs(yaw_rate)
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < turn_duration and not rospy.is_shutdown():
            self.set_velocity(yaw_rate=yaw_rate)
            rospy.sleep(0.1)
        self.set_velocity(yaw_rate=0)
        rospy.loginfo("轉向完成。")

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

    def visual_return_home(self, num_steps=100):
        """
        新的視覺回家流程：
        每次回程拍照後，與去程資料庫中的影像比對，找出最相似的兩張影像，
        根據它們檔名中的GPS資訊與匹配相似度以加權平均(線性內插)求得當下圖的GPS，
        並計算拍照到求出GPS的時間，然後指令無人機朝起點GPS方向自動飛行。
        當估計GPS與起點GPS差距小於5公尺時，觸發降落。
        """
        for step in range(num_steps):
            t_start = time.time()
            img_path = self.save_image(is_returning=True)
            if img_path is None:
                continue

            # 與去程影像比對，找出最相似的兩張影像
            matches, _ = find_most_similar_images(img_path, self.forward_save_path, num_matches=2)
            if not matches:
                rospy.logwarn("匹配不足，無法取得相似影像，繼續下一輪")
                elapsed = time.time() - t_start
                remaining = 10 - elapsed
                if remaining > 0:
                    rospy.sleep(remaining)
                continue

            # 新增：從比對結果中解析出前兩張影像的GPS，並印出10個插值經緯度
            if len(matches) >= 2:
                try:
                    _, file_path1, *_ = matches[0]
                    _, file_path2, *_ = matches[1]
                    parts1 = os.path.splitext(os.path.basename(file_path1))[0].split(',')
                    parts2 = os.path.splitext(os.path.basename(file_path2))[0].split(',')
                    lat1, lon1 = float(parts1[0]), float(parts1[1])
                    lat2, lon2 = float(parts2[0]), float(parts2[1])
                    self.print_interpolated_gps(lat1, lon1, lat2, lon2)
                except Exception as e:
                    rospy.logwarn(f"解析匹配影像GPS時發生錯誤: {e}")

            total_weight = 0.0
            weighted_lat = 0.0
            weighted_lon = 0.0
            for match in matches:
                similarity, file_path, _, _, _, _ = match
                basename = os.path.basename(file_path)
                rospy.loginfo(f"比對到的圖片檔名：{basename}，匹配點數量：{similarity}")
                name_no_ext = os.path.splitext(basename)[0]
                parts = name_no_ext.split(',')
                if len(parts) != 2:
                    rospy.logwarn(f"檔名格式錯誤：{basename}")
                    continue
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                except ValueError:
                    rospy.logwarn(f"檔名GPS解析失敗：{basename}")
                    continue
                weighted_lat += similarity * lat
                weighted_lon += similarity * lon
                total_weight += similarity

            if total_weight == 0:
                rospy.logwarn("所有匹配影像GPS解析失敗")
                continue

            estimated_lat = weighted_lat / total_weight
            estimated_lon = weighted_lon / total_weight
            t_elapsed = time.time() - t_start
            rospy.loginfo(f"[Step {step+1}] 估計當下GPS: ({estimated_lat:.6f}, {estimated_lon:.6f}), 處理時間: {t_elapsed:.2f} 秒")

            # 若已知起點GPS，計算從當前估計GPS到起點的差距（轉換為公尺），當差距小於5公尺時降落
            if self.start_gps is not None:
                start_lat, start_lon = self.start_gps
                delta_lat = start_lat - estimated_lat
                delta_lon = start_lon - estimated_lon
                meters_per_deg_lat = 111320
                meters_per_deg_lon = 111320 * math.cos(math.radians(estimated_lat))
                dx = delta_lat * meters_per_deg_lat
                dy = delta_lon * meters_per_deg_lon
                distance_to_start = math.sqrt(dx**2 + dy**2)
                rospy.loginfo(f"從估計位置到起點差距: dx={dx:.2f} m, dy={dy:.2f} m, total={distance_to_start:.2f} m")
                if distance_to_start < 5:
                    rospy.loginfo("與起點GPS相差小於5m，開始降落")
                    self.land()
                    return

                # 若未達降落條件，則依差距產生速度修正指令
                target_vx = self.shift_kp * dx
                target_vy = self.shift_kp * dy
                max_vel = 3.0
                target_vx = max(min(target_vx, max_vel), -max_vel)
                target_vy = max(min(target_vy, max_vel), -max_vel)

                t0 = time.time()
                ramp_duration = 2.0
                hold_duration = 1.0
                decel_duration = 2.0

                while time.time() - t0 < ramp_duration and not rospy.is_shutdown():
                    factor = (time.time() - t0) / ramp_duration
                    self.set_velocity(vx=factor * target_vx, vy=factor * target_vy, vz=0, yaw_rate=0)
                    rospy.sleep(0.1)

                t_hold = time.time()
                while time.time() - t_hold < hold_duration and not rospy.is_shutdown():
                    self.set_velocity(vx=target_vx, vy=target_vy, vz=0, yaw_rate=0)
                    rospy.sleep(0.1)

                t_decel = time.time()
                while time.time() - t_decel < decel_duration and not rospy.is_shutdown():
                    factor = 1 - ((time.time() - t_decel) / decel_duration)
                    self.set_velocity(vx=factor * target_vx, vy=factor * target_vy, vz=0, yaw_rate=0)
                    rospy.sleep(0.1)

                self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)

            # 補足等待時間，確保每個循環週期至少持續指定秒數
            elapsed = time.time() - t_start
            remaining = 2 - elapsed
            if remaining > 0:
                rospy.sleep(remaining)
        rospy.loginfo("視覺回家流程結束。")

    def fly_and_capture(self, start_x, start_y, end_x, end_y, speed, capture_interval):
        rospy.loginfo("準備起飛...")
        self.takeoff(target_altitude=50)

        rospy.loginfo("開始往前 (XY) 飛行並拍照...")
        self.fly_straight_xy(start_x, start_y, end_x, end_y, speed, capture_interval)

        # 轉向前拍攝一張參考影像
        rospy.loginfo("轉向前拍攝參考影像...")
        reference_image_path = self.save_image(is_returning=False)
        if reference_image_path is not None:
            rospy.loginfo(f"轉向前參考影像已儲存：{reference_image_path}")

        self.turn_around()

        rospy.loginfo("開始視覺回家 (return)...")
        self.visual_return_home(num_steps=100)

        rospy.loginfo("準備降落...")
        self.land()

if __name__ == '__main__':
    try:
        drone = DroneFlyAndCapture()
        while not drone.current_state.connected:
            rospy.loginfo("等待 FCU 連線...")
            rospy.sleep(1)
        # 將 capture_interval 參數設為10秒
        drone.fly_and_capture(
            start_x=0,
            start_y=0,
            end_x=100,
            end_y=100,
            speed=3.0,
            capture_interval=10
        )
    except rospy.ROSInterruptException:
        rospy.loginfo("程式被中斷。")

