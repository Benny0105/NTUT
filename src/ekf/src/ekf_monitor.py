#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, NavSatFix
from std_msgs.msg import UInt32
import csv
import os
import signal
import sys
import datetime

class EkfMonitor:
    def __init__(self):
        rospy.init_node('ekf_monitor', anonymous=True)
        
        # Initialize timestamp variables to avoid None errors
        self.pose_timestamp = rospy.Time(0)
        self.twist_timestamp = rospy.Time(0)
        self.imu_timestamp = rospy.Time(0)
        self.mag_timestamp = rospy.Time(0)
        self.pressure_timestamp = rospy.Time(0)
        self.last_log_time = rospy.Time(0)
        self.raw_odom_timestamp = rospy.Time(0)
        self.gps_timestamp = rospy.Time(0)
        self.gps_satellites_timestamp = rospy.Time(0)
        
        # Get parameters
        self.log_rate = rospy.get_param('~log_rate', 1.0)  # Hz
        self.log_file_path = rospy.get_param('~log_file_path', '~/ekf_log.csv')
        self.use_ned_frame = rospy.get_param('~use_ned_frame', True)
        self.print_frequency = rospy.get_param('~print_frequency', 1.0)  # Hz
        
        # 獲取EKF的傳感器使能參數
        self.use_gps = rospy.get_param('/px4_style_ekf/use_gps', True)
        self.use_mag = rospy.get_param('/px4_style_ekf/use_mag', True)
        self.use_baro = rospy.get_param('/px4_style_ekf/use_baro', True)
        self.use_odom = rospy.get_param('/px4_style_ekf/use_odom', False)
        self.use_gps_vel = rospy.get_param('/px4_style_ekf/use_gps_vel', True)
        
        # 輸出EKF參數設定
        rospy.loginfo("EKF sensor configuration: GPS=%s, MAG=%s, BARO=%s, ODOM=%s, GPS_VEL=%s",
                     "ON" if self.use_gps else "OFF",
                     "ON" if self.use_mag else "OFF",
                     "ON" if self.use_baro else "OFF",
                     "ON" if self.use_odom else "OFF",
                     "ON" if self.use_gps_vel else "OFF")
        
        # Expand user directory and ensure absolute path
        if '~' in self.log_file_path:
            self.log_file_path = os.path.expanduser(self.log_file_path)
        
        # Convert to absolute path if not already
        if not os.path.isabs(self.log_file_path):
            catkin_ws = os.environ.get('ROS_WORKSPACE', '/home/jim/Benny/catkin_ws')
            if os.path.exists(catkin_ws):
                self.log_file_path = os.path.join(catkin_ws, self.log_file_path)
            else:
                # Fall back to current working directory
                self.log_file_path = os.path.join(os.getcwd(), self.log_file_path)
        
        # Add timestamp to filename to avoid overwriting
        log_dir = os.path.dirname(self.log_file_path)
        log_filename = os.path.basename(self.log_file_path)
        log_name, log_ext = os.path.splitext(log_filename)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(log_dir, f"{log_name}_{timestamp}{log_ext}")
        
        # Check if directory exists
        if not os.path.exists(log_dir) and log_dir:
            try:
                os.makedirs(log_dir)
                rospy.loginfo("Created log directory: %s", log_dir)
            except Exception as e:
                rospy.logwarn("Cannot create log directory: %s - %s", log_dir, str(e))
                # Fall back to current working directory
                self.log_file_path = os.path.join(os.getcwd(), f"ekf_log_{timestamp}.csv")
                
        # Log CSV file path
        rospy.loginfo("Using log file path: %s", self.log_file_path)
        
        # Initialize CSV log file
        self.csv_file = None
        self.csv_writer = None
        try:
            self.csv_file = open(self.log_file_path, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            # Update CSV header
            self.csv_writer.writerow([
                'Time', 'Position_X', 'Position_Y', 'Position_Z', 
                'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W',
                'Velocity_X', 'Velocity_Y', 'Velocity_Z',
                'Angular_Velocity_X', 'Angular_Velocity_Y', 'Angular_Velocity_Z',
                'Raw_Gazebo_Odom_X', 'Raw_Gazebo_Odom_Y', 'Raw_Gazebo_Odom_Z',
                'Raw_Gazebo_Odom_Qx', 'Raw_Gazebo_Odom_Qy', 'Raw_Gazebo_Odom_Qz', 'Raw_Gazebo_Odom_Qw',
                'Raw_Gazebo_Odom_Vx', 'Raw_Gazebo_Odom_Vy', 'Raw_Gazebo_Odom_Vz',
                'GPS_Lat', 'GPS_Lon', 'GPS_Alt', 'GPS_Satellites'
            ])
            self.csv_file.flush()  # Force write to disk
            rospy.loginfo("EKF log file initialized: %s", self.log_file_path)
        except Exception as e:
            rospy.logerr("Failed to initialize log file: %s", str(e))
            self.csv_file = None
            
        # Set up termination handling to ensure file is closed properly
        signal.signal(signal.SIGINT, self.signal_handler)
            
        # Initialize data storage
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.mag_field = np.zeros(3)
        self.pressure = 0.0
        
        # Raw odometry data storage
        self.raw_odom_position = np.zeros(3)
        self.raw_odom_orientation = np.zeros(4)
        self.raw_odom_velocity = np.zeros(3)
        self.raw_odom_angular_velocity = np.zeros(3)
        
        # GPS data storage
        self.gps_latitude = 0.0
        self.gps_longitude = 0.0
        self.gps_altitude = 0.0
        self.gps_satellites = 0
        self.gps_fix_status = 0  # 0=No fix, 1=Fix, 2=DGPS, etc.
        
        # Subscribe to EKF output topics
        # Subscribe to different message types
        try:
            self.pose_sub = rospy.Subscriber('/px4_style_ekf/pose', PoseWithCovarianceStamped, self.pose_callback_with_cov)
            rospy.loginfo("Subscribed to /px4_style_ekf/pose as PoseWithCovarianceStamped")
        except Exception as e:
            rospy.logwarn("Unable to subscribe to /px4_style_ekf/pose as PoseWithCovarianceStamped: %s", str(e))
            try:
                self.pose_sub = rospy.Subscriber('/px4_style_ekf/pose', PoseStamped, self.pose_callback)
                rospy.loginfo("Subscribed to /px4_style_ekf/pose as PoseStamped")
            except Exception as e:
                rospy.logwarn("Unable to subscribe to /px4_style_ekf/pose as PoseStamped: %s", str(e))
        
        self.twist_sub = rospy.Subscriber('/px4_style_ekf/velocity', TwistStamped, self.twist_callback)
        self.odom_sub = rospy.Subscriber('/px4_style_ekf/odom', Odometry, self.odom_callback)
        
        # Subscribe to sensor input topics (for monitoring only, not recorded to CSV)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        self.mag_sub = rospy.Subscriber('/mavros/imu/mag', MagneticField, self.mag_callback)
        self.pressure_sub = rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, self.pressure_callback)
        
        # Subscribe to raw odometry topic
        self.raw_odom_sub = rospy.Subscriber('/gazebo/odom_ned', Odometry, self.raw_odom_callback)
        rospy.loginfo("Subscribed to raw odometry topic: /gazebo/odom_ned")
        
        # Subscribe to GPS and satellites topics
        self.gps_sub = rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_callback)
        self.gps_satellites_sub = rospy.Subscriber('/mavros/global_position/raw/satellites', UInt32, self.gps_satellites_callback)
        rospy.loginfo("Subscribed to GPS topics: /mavros/global_position/raw/fix and /mavros/global_position/raw/satellites")
        
        # Create timer to log data
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.log_rate), self.log_data)
        
        rospy.loginfo("EKF Monitor initialized - Using NED coordinate system: %s", "Yes" if self.use_ned_frame else "No")
        
    def signal_handler(self, sig, frame):
        """Handle termination signal to ensure file is closed properly"""
        rospy.loginfo("Received termination signal, closing log file...")
        self.close_csv_file()
        sys.exit(0)
        
    def close_csv_file(self):
        """Safely close CSV file"""
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
                rospy.loginfo("Successfully closed log file: %s", self.log_file_path)
            except Exception as e:
                rospy.logerr("Error closing log file: %s", str(e))
            self.csv_file = None
        
    def pose_callback(self, msg):
        self.pose_timestamp = msg.header.stamp
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z
        self.orientation[0] = msg.pose.orientation.x
        self.orientation[1] = msg.pose.orientation.y
        self.orientation[2] = msg.pose.orientation.z
        self.orientation[3] = msg.pose.orientation.w
        
    def pose_callback_with_cov(self, msg):
        self.pose_timestamp = msg.header.stamp
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        
    def twist_callback(self, msg):
        self.twist_timestamp = msg.header.stamp
        self.velocity[0] = msg.twist.linear.x
        self.velocity[1] = msg.twist.linear.y
        self.velocity[2] = msg.twist.linear.z
        self.angular_velocity[0] = msg.twist.angular.x
        self.angular_velocity[1] = msg.twist.angular.y
        self.angular_velocity[2] = msg.twist.angular.z
        
    def odom_callback(self, msg):
        # Update position, orientation and velocity using odometry message
        self.pose_timestamp = msg.header.stamp
        self.twist_timestamp = msg.header.stamp
        
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        
        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w
        
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y
        self.velocity[2] = msg.twist.twist.linear.z
        
        self.angular_velocity[0] = msg.twist.twist.angular.x
        self.angular_velocity[1] = msg.twist.twist.angular.y
        self.angular_velocity[2] = msg.twist.twist.angular.z
        
    def raw_odom_callback(self, msg):
        self.raw_odom_timestamp = msg.header.stamp
        
        # Store raw odometry data
        self.raw_odom_position[0] = msg.pose.pose.position.x
        self.raw_odom_position[1] = msg.pose.pose.position.y
        self.raw_odom_position[2] = msg.pose.pose.position.z
        
        self.raw_odom_orientation[0] = msg.pose.pose.orientation.x
        self.raw_odom_orientation[1] = msg.pose.pose.orientation.y
        self.raw_odom_orientation[2] = msg.pose.pose.orientation.z
        self.raw_odom_orientation[3] = msg.pose.pose.orientation.w
        
        self.raw_odom_velocity[0] = msg.twist.twist.linear.x
        self.raw_odom_velocity[1] = msg.twist.twist.linear.y
        self.raw_odom_velocity[2] = msg.twist.twist.linear.z
        
        self.raw_odom_angular_velocity[0] = msg.twist.twist.angular.x
        self.raw_odom_angular_velocity[1] = msg.twist.twist.angular.y
        self.raw_odom_angular_velocity[2] = msg.twist.twist.angular.z
    
    def gps_callback(self, msg):
        self.gps_timestamp = msg.header.stamp
        self.gps_latitude = msg.latitude
        self.gps_longitude = msg.longitude
        self.gps_altitude = msg.altitude
        self.gps_fix_status = msg.status.status
    
    def gps_satellites_callback(self, msg):
        self.gps_satellites_timestamp = rospy.Time.now()
        self.gps_satellites = msg.data
        
    def imu_callback(self, msg):
        self.imu_timestamp = msg.header.stamp
        self.angular_velocity[0] = msg.angular_velocity.x
        self.angular_velocity[1] = msg.angular_velocity.y
        self.angular_velocity[2] = msg.angular_velocity.z
        
        self.linear_acceleration[0] = msg.linear_acceleration.x
        self.linear_acceleration[1] = msg.linear_acceleration.y
        self.linear_acceleration[2] = msg.linear_acceleration.z
        
    def mag_callback(self, msg):
        self.mag_timestamp = msg.header.stamp
        self.mag_field[0] = msg.magnetic_field.x
        self.mag_field[1] = msg.magnetic_field.y
        self.mag_field[2] = msg.magnetic_field.z
        
    def pressure_callback(self, msg):
        self.pressure_timestamp = msg.header.stamp
        self.pressure = msg.fluid_pressure
        
    def log_data(self, event):
        now = rospy.Time.now()
        
        # Check data validity
        time_threshold = rospy.Duration(2.0)  # 2 second threshold
        pose_age = now - self.pose_timestamp if self.pose_timestamp else rospy.Duration(9999)
        imu_age = now - self.imu_timestamp if self.imu_timestamp else rospy.Duration(9999)
        mag_age = now - self.mag_timestamp if self.mag_timestamp else rospy.Duration(9999)
        pressure_age = now - self.pressure_timestamp if self.pressure_timestamp else rospy.Duration(9999)
        raw_odom_age = now - self.raw_odom_timestamp if self.raw_odom_timestamp else rospy.Duration(9999)
        gps_age = now - self.gps_timestamp if self.gps_timestamp else rospy.Duration(9999)
        gps_satellites_age = now - self.gps_satellites_timestamp if self.gps_satellites_timestamp else rospy.Duration(9999)
        
        # Check data delay and print warnings
        if pose_age > time_threshold:
            rospy.logwarn("EKF pose data delay: %.2f seconds (/px4_style_ekf/pose)", pose_age.to_sec())
            
        if imu_age > time_threshold:
            rospy.logwarn("IMU data delay: %.2f seconds (/mavros/imu/data)", imu_age.to_sec())
            
        if mag_age > time_threshold:
            rospy.logwarn("Magnetometer data delay: %.2f seconds (/mavros/imu/mag)", mag_age.to_sec())
            
        if pressure_age > time_threshold:
            rospy.logwarn("Barometer data delay: %.2f seconds (/mavros/imu/static_pressure)", pressure_age.to_sec())
            
        if raw_odom_age > time_threshold:
            rospy.logwarn("Raw odometry data delay: %.2f seconds (/gazebo/odom_ned)", raw_odom_age.to_sec())
            
        if gps_age > time_threshold:
            rospy.logwarn("GPS data delay: %.2f seconds (/mavros/global_position/raw/fix)", gps_age.to_sec())
            
        if gps_satellites_age > time_threshold:
            rospy.logwarn("GPS satellites data delay: %.2f seconds (/mavros/global_position/raw/satellites)", gps_satellites_age.to_sec())
        
        # Write data to CSV (including EKF results and raw odometry data)
        if self.csv_file and self.csv_writer:
            try:
                self.csv_writer.writerow([
                    now.to_sec(),
                    self.position[0], self.position[1], self.position[2],
                    self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3],
                    self.velocity[0], self.velocity[1], self.velocity[2],
                    self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2],
                    # Raw odometry data
                    self.raw_odom_position[0], self.raw_odom_position[1], self.raw_odom_position[2],
                    self.raw_odom_orientation[0], self.raw_odom_orientation[1], 
                    self.raw_odom_orientation[2], self.raw_odom_orientation[3],
                    self.raw_odom_velocity[0], self.raw_odom_velocity[1], self.raw_odom_velocity[2],
                    # GPS data
                    self.gps_latitude, self.gps_longitude, self.gps_altitude, self.gps_satellites
                ])
                # Force write to disk every 10 logs to balance performance and safety
                if int(now.to_sec() * self.log_rate) % 10 == 0:
                    self.csv_file.flush()
            except Exception as e:
                rospy.logerr("Failed to write to log file: %s", str(e))
                # Try to reopen the file
                self.close_csv_file()
                try:
                    self.csv_file = open(self.log_file_path, 'a')
                    self.csv_writer = csv.writer(self.csv_file)
                    rospy.loginfo("Reopened log file")
                except:
                    rospy.logerr("Cannot reopen log file")
        
        # Print EKF status once per second - CONSOLIDATED FORMAT
        if now - self.last_log_time >= rospy.Duration(1.0 / self.print_frequency):
            self.last_log_time = now
            
            coord_system = "NED" if self.use_ned_frame else "ENU"
            
            # Prepare consolidated status message with clear section headers
            rospy.loginfo("----------------- EKF STATUS REPORT -----------------")
            
            # Sensor health status - 現在根據EKF參數顯示不同狀態
            sensor_health = (
                f"SENSOR HEALTH STATUS:"
                f"\n  IMU:      {('OK' if imu_age.to_sec() < 1.0 else f'DELAY {imu_age.to_sec():.1f}s')}"
                f"\n  MAG:      {('OK' if mag_age.to_sec() < 1.0 and self.use_mag else 'DISABLED') if self.use_mag else 'DISABLED'}"
                f"\n  BARO:     {('OK' if pressure_age.to_sec() < 1.0 and self.use_baro else 'DISABLED') if self.use_baro else 'DISABLED'}"
                f"\n  ODOMETRY: {('OK' if raw_odom_age.to_sec() < 1.0 and self.use_odom else 'DISABLED') if self.use_odom else 'DISABLED'}"
                f"\n  GPS:      {('OK' if gps_age.to_sec() < 1.0 and self.use_gps else 'DISABLED') if self.use_gps else 'DISABLED'} (Satellites: {self.gps_satellites}, Fix: {self.get_fix_status_text()})"
            )
            rospy.loginfo(sensor_health)
            
            # EKF Position/Velocity
            axis_labels = ["N", "E", "D"] if self.use_ned_frame else ["X", "Y", "Z"]
            ekf_status = (
                f"EKF ESTIMATION ({coord_system}):"
                f"\n  Position:    {axis_labels[0]}={self.position[0]:7.2f}m  {axis_labels[1]}={self.position[1]:7.2f}m  {axis_labels[2]}={self.position[2]:7.2f}m"
                f"\n  Velocity:    {axis_labels[0]}={self.velocity[0]:7.2f}m/s {axis_labels[1]}={self.velocity[1]:7.2f}m/s {axis_labels[2]}={self.velocity[2]:7.2f}m/s"
            )
            rospy.loginfo(ekf_status)
            
            # Raw Odometry Data
            raw_odom_status = (
                f"{'RAW ODOMETRY DATA (DISABLED IN EKF)' if not self.use_odom else f'RAW ODOMETRY ({coord_system})'}:"
                f"\n  Position:    {axis_labels[0]}={self.raw_odom_position[0]:7.2f}m  {axis_labels[1]}={self.raw_odom_position[1]:7.2f}m  {axis_labels[2]}={self.raw_odom_position[2]:7.2f}m"
                f"\n  Velocity:    {axis_labels[0]}={self.raw_odom_velocity[0]:7.2f}m/s {axis_labels[1]}={self.raw_odom_velocity[1]:7.2f}m/s {axis_labels[2]}={self.raw_odom_velocity[2]:7.2f}m/s"
            )
            rospy.loginfo(raw_odom_status)
            
            # GPS Data
            gps_status = (
                f"{'GPS DATA (DISABLED IN EKF)' if not self.use_gps else 'GPS DATA'}:"
                f"\n  Position:    Lat={self.gps_latitude:11.7f}°  Lon={self.gps_longitude:11.7f}°  Alt={self.gps_altitude:7.2f}m"
                f"\n  Satellites:  {self.gps_satellites:3d}  Fix Status: {self.get_fix_status_text()}"
            )
            rospy.loginfo(gps_status)
            
            # EKF-Odometry Difference
            diff_status = (
                f"EKF-ODOMETRY DIFFERENCE:"
                f"\n  Position:    {axis_labels[0]}={self.position[0]-self.raw_odom_position[0]:7.2f}m  {axis_labels[1]}={self.position[1]-self.raw_odom_position[1]:7.2f}m  {axis_labels[2]}={self.position[2]-self.raw_odom_position[2]:7.2f}m"
                f"\n  Velocity:    {axis_labels[0]}={self.velocity[0]-self.raw_odom_velocity[0]:7.2f}m/s {axis_labels[1]}={self.velocity[1]-self.raw_odom_velocity[1]:7.2f}m/s {axis_labels[2]}={self.velocity[2]-self.raw_odom_velocity[2]:7.2f}m/s"
            )
            rospy.loginfo(diff_status)
            
            # Data logging status
            if self.csv_file:
                rospy.loginfo(f"DATA LOGGING: Active - {self.log_file_path}")
            else:
                rospy.loginfo("DATA LOGGING: Inactive - No log file")
                
            rospy.loginfo("-----------------------------------------------------")
    
    def get_fix_status_text(self):
        """Convert GPS fix status code to text"""
        if self.gps_fix_status == -1:  # Custom status
            return "INVALID"
        elif self.gps_fix_status == 0:
            return "NO FIX"
        elif self.gps_fix_status == 1:
            return "FIX"
        elif self.gps_fix_status == 2:
            return "DGPS"
        elif self.gps_fix_status == 3:
            return "RTK FLOAT"
        elif self.gps_fix_status == 4:
            return "RTK FIX"
        else:
            return f"UNKNOWN ({self.gps_fix_status})"
    
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("User interrupted monitor node...")
        finally:
            self.close_csv_file()
            rospy.loginfo("EKF Monitor node closed")

if __name__ == '__main__':
    try:
        monitor = EkfMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass 