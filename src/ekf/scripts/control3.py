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
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

# 匯入 feature 模組的函式
from feature2 import (
    compute_dx_dy_between_images,
)

class DroneFlyAndCapture:
    def __init__(self):
        rospy.init_node('drone_fly_and_capture_ekf', anonymous=True)

        # ----------------- 影像存檔資料夾 -----------------
        self.forward_save_path = "/home/jim/gazebo_images/forward/"
        self.return_save_path = "/home/jim/gazebo_images/return/"
        os.makedirs(self.forward_save_path, exist_ok=True)
        os.makedirs(self.return_save_path, exist_ok=True)

        # 影像轉換工具
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/iris_vision/usb_cam/image_raw",
            Image,
            self.image_callback
        )

        # ----------------- ROS Publisher 與 Service -----------------
        self.pose_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )
        self.velocity_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel',
            TwistStamped,
            queue_size=10
        )

        # 訂閱 EKF 輸出的位姿和狀態
        rospy.Subscriber(
            '/px4_style_ekf/pose',
            PoseWithCovarianceStamped,
            self.ekf_pose_callback
        )
        rospy.Subscriber(
            '/ekf/status',
            State,
            self.ekf_status_callback
        )

        # 等待並連接 MAVROS 解鎖與模式切換服務
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_service = rospy.ServiceProxy(
            '/mavros/cmd/arming',
            CommandBool
        )
        self.set_mode_service = rospy.ServiceProxy(
            '/mavros/set_mode',
            SetMode
        )

        # 嘗試連接 GPS 控制服務
        rospy.loginfo("嘗試連接GPS控制服務...")
        self.gps_control_service = None
        try:
            rospy.loginfo("等待GPS控制服務啟動（最多10秒）...")
            rospy.wait_for_service('/gps_control_node/enable_gps', timeout=10.0)
            self.gps_control_service = rospy.ServiceProxy(
                '/gps_control_node/enable_gps',
                SetBool
            )
            rospy.loginfo("GPS控制服務已連接")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"無法連接GPS控制服務：{e}，返航時將無法關閉GPS")

        # 訂閱無人機狀態、位置、GPS
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        # ----------------- 狀態變數 -----------------
        self.current_state = State()
        self.current_position = PoseStamped()
        self.current_gps = None
        self.ekf_position = None
        self.ekf_status = None
        self.current_image = None
        # 將主迴圈頻率調高到 50Hz
        self.rate = rospy.Rate(50)

        # 儲存起點位姿資訊
        self.start_position = None     # (x, y, z) from EKF
        self.start_gps = None          # (lat, lon)
        self.start_image_path = None   # 起飛時的參考影像路徑

        # 可調增益參數 (提高角度增益)
        self.shift_kp = 0.5    # 位移補償增益
        self.angle_kp = 1.0    # 由 0.5 提升至 1.0
        self.position_kp = 1.0 # 位置控制增益

        # Adaptive Heading Bias 參數
        self.yaw_bias = 0.0           # 偏置累積量
        self.Ki_yaw = 0.01            # 偏置累積增益

        rospy.loginfo("無人機控制節點已初始化，等待其他系統...")

    # ---------------- 回調與輔助函式 ----------------
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
    def ekf_pose_callback(self, msg):
        self.ekf_position = msg
    def ekf_status_callback(self, msg):
        self.ekf_status = msg

    def get_current_yaw(self):
        q = self.current_position.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)

    def get_ekf_yaw(self):
        if self.ekf_position is None:
            return 0.0
        q = self.ekf_position.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0*math.pi
        while angle < -math.pi:
            angle += 2.0*math.pi
        return angle

    def get_ekf_position(self):
        if self.ekf_position is None:
            return None
        p = self.ekf_position.pose.pose.position
        return (p.x, p.y, p.z)

    # --------- 飛行控制相關 ---------
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
        """ 發 ENU 座標下的速度：linear(x,y,z)+航向速率 """
        msg = TwistStamped()
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.z = yaw_rate
        msg.header.stamp = rospy.Time.now()
        self.velocity_pub.publish(msg)

    # --- 新增：用機體座標下的前進/側向 → 轉 ENU 發布 ---
    def set_body_velocity(self, vx_body, vy_body, vz, yaw_rate):
        """
        vx_body: 機頭朝前速度
        vy_body: 機頭朝右速度
        """
        cyaw = self.ned_to_enu(self.get_ekf_yaw())
        vx = vx_body * math.cos(cyaw) - vy_body * math.sin(cyaw)
        vy = vx_body * math.sin(cyaw) + vy_body * math.cos(cyaw)
        self.set_velocity(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def takeoff(self, target_altitude=50):
        # 先預熱 velocity setpoints
        for _ in range(100):
            self.set_velocity(0,0,0,0)
            rospy.sleep(0.05)

        rospy.loginfo(f"起飛至 {target_altitude} m 高度...")
        if not self.arm(): return
        if not self.set_offboard_mode(): return
        while self.current_position.pose.position.z < target_altitude - 1 and not rospy.is_shutdown():
            self.set_velocity(vz=5.0)
            rospy.sleep(0.2)
        self.set_velocity(vz=0)
        rospy.loginfo(f"成功達到 {target_altitude} m！")

        # 記錄起點 EKF / GPS 及參考影像
        self.start_position = self.get_ekf_position()
        rospy.loginfo(f"已記錄起點位置 (EKF): {self.start_position}")
        if self.current_gps:
            self.start_gps = (self.current_gps.latitude, self.current_gps.longitude)
            rospy.loginfo(f"起點GPS: {self.start_gps}")
        p = self.save_image(is_returning=False)
        if p:
            rospy.loginfo(f"起點影像已儲存：{p}")
            self.start_image_path = p

    def land(self):
        rospy.loginfo("開始降落 (AUTO.LAND)...")
        self.set_mode_service(custom_mode="AUTO.LAND")
        rospy.loginfo("無人機正在降落。")

    def ned_to_enu(self, yaw_ned):
        """ NED → ENU 轉 yaw """
        return self.normalize_angle((math.pi/2) - yaw_ned)

    def turn_to_start_point(self, yaw_rate_max=2.0, error_threshold=0.1,
                            max_time=30.0, Kp=0.4, Kd=0.12):
        if self.start_position is None:
            rospy.logerr("尚未記錄起點位置，無法 turn_to_start_point")
            return
        cur = self.get_ekf_position()
        if cur is None:
            rospy.logerr("無法取得 EKF 位置")
            return

        dx = self.start_position[0] - cur[0]
        dy = self.start_position[1] - cur[1]
        target_ned = math.atan2(dy, dx)
        target_enu = self.ned_to_enu(target_ned)

        cur_ned = self.get_ekf_yaw()
        cur_enu = self.ned_to_enu(cur_ned)
        err = self.normalize_angle(target_enu - cur_enu)
        rospy.loginfo(f"計算返航轉向 (ENU): 目標角度={math.degrees(target_enu):.1f}, 初始誤差={math.degrees(err):.1f} deg")

        self.set_velocity(0,0,0,0); rospy.sleep(1)
        t0 = rospy.Time.now().to_sec(); prev_t = t0; prev_e = err

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = now - prev_t if now>prev_t else 0.02
            cur_enu = self.ned_to_enu(self.get_ekf_yaw())
            err = self.normalize_angle(target_enu - cur_enu)
            if abs(err) < error_threshold:
                break
            deriv = (err - prev_e)/dt
            yaw_rate = Kp*err + Kd*deriv
            yaw_rate = max(-yaw_rate_max, min(yaw_rate_max, yaw_rate))
            if abs(yaw_rate) < 0.05:
                yaw_rate = 0.0
            self.set_velocity(0,0,0,yaw_rate)
            prev_e = err; prev_t = now
            rospy.sleep(0.02)
            if now - t0 > max_time:
                rospy.logwarn("轉向超時，強制退出")
                break

        self.set_velocity(0,0,0,0); rospy.sleep(0.5)
        rospy.loginfo("轉向完成")

    def save_image(self, is_returning=False):
        if self.current_image is None:
            rospy.logwarn("無法儲存影像：尚未收到任何影像!")
            return None
        if is_returning:
            count = len([f for f in os.listdir(self.return_save_path) if f.endswith('.jpg')])
            fn = f"{count+1}.jpg"; path = os.path.join(self.return_save_path, fn)
        else:
            if self.current_gps:
                fn = f"{self.current_gps.latitude:.6f},{self.current_gps.longitude:.6f}.jpg"
            else:
                count = len([f for f in os.listdir(self.forward_save_path) if f.endswith('.jpg')])
                fn = f"{count+1}.jpg"
            path = os.path.join(self.forward_save_path, fn)
        cv2.imwrite(path, self.current_image)
        rospy.loginfo(f"影像已儲存：{path}")
        return path

    def fly_straight_xy(self, start_x, start_y, end_x, end_y, speed, capture_interval):
        init_x = self.current_position.pose.position.x
        init_y = self.current_position.pose.position.y
        dx = end_x - init_x; dy = end_y - init_y
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            rospy.logwarn("目標距離太短，不執行飛行"); return

        heading = math.atan2(dy, dx)
        vx = speed*math.cos(heading)
        vy = speed*math.sin(heading)

        traveled = 0.0; last_cap = time.time()
        rospy.loginfo(f"開始飛行至 x={end_x:.1f}, y={end_y:.1f}, heading={heading:.2f}")
        rate = rospy.Rate(20)
        while traveled < dist and not rospy.is_shutdown():
            cyaw = self.get_current_yaw()
            err = self.normalize_angle(heading - cyaw)
            yaw_rate = self.angle_kp * err
            self.set_velocity(vx=vx, vy=vy, vz=0, yaw_rate=yaw_rate)

            cx = self.current_position.pose.position.x
            cy = self.current_position.pose.position.y
            traveled = math.hypot(cx-init_x, cy-init_y)

            if time.time() - last_cap >= capture_interval:
                self.save_image(is_returning=False)
                last_cap = time.time()
            rate.sleep()

        self.set_velocity(0,0,0,0)
        rospy.loginfo("已到達目標或超過距離")

    def ekf_return_home(self, max_duration=1000.0, check_interval=0.5):
        """使用機體座標同時前進＋轉向返航，加入 D 項提升機頭穩定性，並線上補償偏置"""
        rospy.loginfo("開始使用 EKF 返航...")
        if self.start_position is None:
            rospy.logerr("無法返航：起點未設定"); return
        if not self.start_image_path:
            rospy.logwarn("起點影像不存在，僅採用 EKF 返航")

        # PID 參數（加強 P、加入 D）
        Kp_yaw = 1.2
        Kd_yaw = 0.2
        Ki_yaw = self.Ki_yaw
        yaw_max = 2.0
        max_spd = 5.0

        # 計算固定的目標航向 once
        pos0 = self.get_ekf_position()
        if pos0 is None:
            rospy.logerr("無法取得 EKF 位置以計算 target_ned"); return
        sx, sy, _ = self.start_position
        cx0, cy0, _ = pos0
        target_ned = math.atan2(sy - cy0, sx - cx0)

        prev_biased_err = 0.0
        prev_time = time.time()

        rate = rospy.Rate(50)
        last_chk = time.time()
        t0 = time.time()

        while not rospy.is_shutdown():
            now = time.time()
            if now - t0 > max_duration:
                rospy.logwarn("超過最大返航時間，退出"); break

            pos = self.get_ekf_position()
            if pos is None:
                rate.sleep(); continue

            sx, sy, _ = self.start_position
            cx, cy, _ = pos
            dx = sx - cx; dy = sy - cy
            dist = math.hypot(dx, dy)
            rospy.loginfo(f"EKF 返航: dx={dx:.2f}, dy={dy:.2f}, dist={dist:.2f}")

            if dist < 5.0:
                rospy.loginfo("到達起點 (<5m)，開始降落！")
                self.land(); return

            # 計算原始誤差
            now_dt = now - prev_time if now>prev_time else 0.02
            yaw_err = self.normalize_angle(
                self.ned_to_enu(target_ned) - self.ned_to_enu(self.get_ekf_yaw())
            )

            # 更新偏置
            self.yaw_bias += Ki_yaw * yaw_err * now_dt
            # 修正後的誤差
            biased_err = self.normalize_angle(yaw_err - self.yaw_bias)

            # 計算 D 項
            deriv = (biased_err - prev_biased_err) / now_dt
            # PD 控制
            yaw_cmd = Kp_yaw * biased_err + Kd_yaw * deriv
            yaw_cmd = max(-yaw_max, min(yaw_max, yaw_cmd))

            prev_biased_err = biased_err
            prev_time = now

            rospy.loginfo(f"[return] yaw_err={math.degrees(biased_err):.1f}°, yaw_cmd={yaw_cmd:.2f}")

            # 速度分解
            spd = min(dist * 0.3, max_spd)
            vx_body = spd * math.cos(biased_err)
            vy_body = spd * math.sin(biased_err)
            self.set_body_velocity(
                vx_body=vx_body,
                vy_body=vy_body,
                vz=0,
                yaw_rate=yaw_cmd
            )

            # 視覺判定降落
            if self.start_image_path and (now - last_chk >= check_interval):
                img = self.save_image(is_returning=True)
                if img:
                    dxv, dyv = compute_dx_dy_between_images(img, self.start_image_path)
                    if dxv is not None and dyv is not None:
                        dv = math.hypot(dxv, dyv)
                        rospy.loginfo(f"[視覺判斷] 與起點: dx={dxv:.2f}, dy={dyv:.2f}, dist={dv:.2f}m")
                        if dv < 15.0:
                            rospy.loginfo("視覺判定到達 (<15m)，降落！")
                            self.land(); return
                last_chk = now

            rate.sleep()

        rospy.loginfo("EKF返航結束，未降落。")

if __name__ == '__main__':
    try:
        drone = DroneFlyAndCapture()
        while not drone.current_state.connected:
            rospy.loginfo("等待 FCU 連線…")
            rospy.sleep(1)

        # --- 定義多個 ENU 座標 waypoint (x, y) ---
        waypoints = [
            (10, 0)
        ]

        # 起飛
        drone.takeoff(target_altitude=50)

        # （可選）開啟 GPS 融合
        if drone.gps_control_service:
            try:
                res = drone.gps_control_service(True)
                if res.success:
                    rospy.loginfo(f"GPS 開啟成功: {res.message}")
                else:
                    rospy.logwarn(f"GPS 開啟失敗: {res.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"GPS 控制呼叫失敗: {e}")
        else:
            rospy.logwarn("無 GPS 控制服務")

        # 依序飛過每個 waypoint
        for idx, (x, y) in enumerate(waypoints):
            rospy.loginfo(f"飛往 waypoint {idx+1}: x={x}, y={y}")
            drone.fly_straight_xy(0, 0, x, y, speed=5.0, capture_interval=10)
            rospy.sleep(1.0)

        # 返回起點並返航、降落
        drone.turn_to_start_point()
        if drone.gps_control_service:
            rospy.loginfo("關閉GPS輸入到EKF...")
            try:
                res = drone.gps_control_service(False)
                if res.success:
                    rospy.loginfo(f"GPS關閉成功: {res.message}")
                else:
                    rospy.logwarn(f"GPS關閉失敗: {res.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"GPS控制呼叫失敗: {e}")
        else:
            rospy.logwarn("無 GPS 控制服務")
        rospy.sleep(1.0)

        drone.ekf_return_home(max_duration=1000.0, check_interval=1.0)
        rospy.loginfo("準備降落…")
        drone.land()

    except rospy.ROSInterruptException:
        rospy.loginfo("程式被中斷。")