#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>  // 添加UInt32消息類型
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>  // 為文件輸出添加頭文件
#include <iomanip>  // 為格式化輸出添加頭文件
#include <cmath>

/**
 * Simplified PX4-style EKF node, designed to receive PX4 Gazebo simulation data and perform position estimation
 * All estimation results are output in the terminal
 */
class PX4EKFSimple {
public:
    PX4EKFSimple() : nh_(""), private_nh_("~") {
        // Get parameters
        private_nh_.param<double>("frequency", frequency_, 50.0);
        private_nh_.param<double>("sensor_timeout", sensor_timeout_, 1.0);
        private_nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
        private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        private_nh_.param<bool>("use_mag", use_mag_, true);
        private_nh_.param<bool>("use_gps", use_gps_, true);
        private_nh_.param<bool>("use_baro", use_baro_, true);
        private_nh_.param<bool>("publish_tf", publish_tf_, true);
        private_nh_.param<bool>("log_data", log_data_, true);
        private_nh_.param<std::string>("log_file_path", log_file_path_, "/home/benny/catkin_ws/ekf_log.csv");
        private_nh_.param<bool>("use_ned_frame", use_ned_frame_, true);
        private_nh_.param<double>("static_pressure_timeout", static_pressure_timeout_, 5.0);
        private_nh_.param<double>("baro_noise", baro_noise_, 0.5);
        private_nh_.param<double>("mag_noise", mag_noise_, 0.01);
        private_nh_.param<double>("gps_noise_lat_lon", gps_noise_lat_lon_, 2.0);  // 緯度/經度噪聲(米)
        private_nh_.param<double>("gps_noise_alt", gps_noise_alt_, 5.0);          // 高度噪聲(米)
        private_nh_.param<double>("gps_pos_innov_gate", gps_pos_innov_gate_, 3.0); // GPS位置創新閾值
        private_nh_.param<int>("min_gps_satellites", min_gps_satellites_, 6);     // 最小可用衛星數量
        private_nh_.param<bool>("use_gps_vel", use_gps_vel_, true);               // 是否使用GPS速度數據
        private_nh_.param<bool>("use_odom", use_odom_, false);                     // 是否使用里程計數據
        private_nh_.param<double>("odom_pos_noise", odom_pos_noise_, 0.1);         // 里程計位置噪聲
        private_nh_.param<double>("odom_vel_noise", odom_vel_noise_, 0.1);        // 里程計速度噪聲
        private_nh_.param<double>("odom_att_noise", odom_att_noise_, 0.02);        // 里程計姿態噪聲
        private_nh_.param<double>("odom_innov_gate", odom_innov_gate_, 3.0);      // 里程計創新閾值
        private_nh_.param<double>("pos_innov_gate", pos_innov_gate_, 5.0);        // 位置創新閾值 (新增)
        private_nh_.param<double>("vel_innov_gate", vel_innov_gate_, 3.0);        // 速度創新閾值 (新增)
        private_nh_.param<bool>("use_odom_height", use_odom_height_, false);      // 是否使用里程計高度數據

        // Initialize data logging
        if (log_data_) {
            initDataLogging();
        }

        // Initialize subscribers - Use topic names without full paths to support remapping
        imu_sub_ = nh_.subscribe("imu_data", 10, &PX4EKFSimple::imuCallback, this);
        if (use_gps_) {
            // Subscribe to GPS data
            gps_sub_ = nh_.subscribe("gps_data", 10, &PX4EKFSimple::gpsCallback, this);
            // Add subscriptions for GPS velocity and satellite count
            if (use_gps_vel_) {
                gps_vel_sub_ = nh_.subscribe("gps_vel_data", 10, &PX4EKFSimple::gpsVelCallback, this);
            }
            gps_sat_sub_ = nh_.subscribe("gps_sat_data", 10, &PX4EKFSimple::gpsSatCallback, this);
        }
        if (use_mag_) {
            mag_sub_ = nh_.subscribe("mag_data", 10, &PX4EKFSimple::magCallback, this);
        }
        if (use_baro_) {
            baro_sub_ = nh_.subscribe("baro_data", 10, &PX4EKFSimple::baroCallback, this);
        }
        if (use_odom_) {
            odom_sub_ = nh_.subscribe("odom_data", 10, &PX4EKFSimple::odomCallback, this);
        }

        // Initialize publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/px4_style_ekf/pose", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/px4_style_ekf/odom", 10);
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/px4_style_ekf/velocity", 10);

        // Initialize EKF state
        initializeState();

        // Set timer, frequency controlled by parameter
        update_timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), 
                                        &PX4EKFSimple::updateCallback, this);

        ROS_INFO("=== EKF node startup ===");
        ROS_INFO("Update frequency: %.1f Hz", frequency_);
        ROS_INFO("Sensor timeout: %.2f s", sensor_timeout_);
        ROS_INFO("Use magnetometer: %s", use_mag_ ? "Y" : "N");
        ROS_INFO("Use GPS: %s", use_gps_ ? "Y" : "N");
        ROS_INFO("Use GPS velocity: %s", use_gps_vel_ ? "Y" : "N");
        ROS_INFO("Min GPS satellites: %d", min_gps_satellites_);
        ROS_INFO("Use barometer: %s", use_baro_ ? "Y" : "N");
        ROS_INFO("Publish TF: %s", publish_tf_ ? "Y" : "N");
        ROS_INFO("Use odometry: %s", use_odom_ ? "Y" : "N");
        if (log_data_) {
            ROS_INFO("Data logging enabled: %s", log_file_path_.c_str());
        }
        ROS_INFO("Waiting for IMU data...");
        ROS_INFO("Odometry innovation gate: %.2f", odom_innov_gate_);
        ROS_INFO("Position innovation gate: %.2f", pos_innov_gate_);
        ROS_INFO("Velocity innovation gate: %.2f", vel_innov_gate_);
        ROS_INFO("Use odometry height: %s", use_odom_height_ ? "Y" : "N");
    }

    ~PX4EKFSimple() {
        // 關閉數據日誌
        if (log_file_.is_open()) {
            log_file_.close();
            ROS_INFO("EKF data log closed");
        }
    }

private:
    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Parameters
    double frequency_;
    double sensor_timeout_;
    std::string map_frame_id_;
    std::string base_frame_id_;
    bool use_mag_;
    bool use_gps_;
    bool use_baro_;
    bool publish_tf_;
    bool log_data_;             // Whether to log data
    std::string log_file_path_; // Log file path
    bool use_ned_frame_;
    double static_pressure_timeout_;
    double baro_noise_;
    double mag_noise_;
    // 新增加的GPS參數
    double gps_noise_lat_lon_;  // 緯度/經度噪聲(米)
    double gps_noise_alt_;      // 高度噪聲(米)
    double gps_pos_innov_gate_; // GPS位置創新閾值
    int min_gps_satellites_;    // 最小可用衛星數量
    bool use_gps_vel_;          // 是否使用GPS速度數據
    // 新增里程計參數
    bool use_odom_;             // 是否使用里程計數據
    double odom_pos_noise_;     // 里程計位置噪聲
    double odom_vel_noise_;     // 里程計速度噪聲
    double odom_att_noise_;     // 里程計姿態噪聲
    double odom_innov_gate_;    // 里程計創新閾值
    double pos_innov_gate_;     // 位置創新閾值 (新增)
    double vel_innov_gate_;     // 速度創新閾值 (新增)
    bool use_odom_height_;      // 是否使用里程計高度數據

    // 數據記錄
    std::ofstream log_file_;
    ros::Time log_start_time_;  // 記錄開始時間

    // 訂閱者
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber mag_sub_;
    ros::Subscriber baro_sub_;
    // 新增的GPS訂閱
    ros::Subscriber gps_vel_sub_;  // GPS速度數據
    ros::Subscriber gps_sat_sub_;  // GPS衛星數量
    // 新增里程計訂閱
    ros::Subscriber odom_sub_;     // 里程計數據

    // 發布者
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher velocity_pub_;

    // 計時器
    ros::Timer update_timer_;

    // TF廣播
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 傳感器數據
    sensor_msgs::Imu latest_imu_;
    sensor_msgs::NavSatFix latest_gps_;
    sensor_msgs::MagneticField latest_mag_;
    sensor_msgs::FluidPressure latest_baro_;
    // 新增的GPS數據
    geometry_msgs::TwistStamped latest_gps_vel_;  // GPS速度
    int latest_gps_satellites_ = 0;               // GPS衛星數量
    // 新增里程計數據
    nav_msgs::Odometry latest_odom_;              // 里程計數據

    // 傳感器數據時間戳
    ros::Time last_imu_time_;
    ros::Time last_gps_time_;
    ros::Time last_mag_time_;
    ros::Time last_baro_time_;
    ros::Time last_gps_vel_time_;  // 新增GPS速度時間戳
    ros::Time last_odom_time_;     // 新增里程計時間戳

    // 傳感器健康狀態
    bool imu_healthy_;
    bool gps_healthy_;
    bool mag_healthy_;
    bool baro_healthy_;
    bool gps_vel_healthy_ = false;  // 新增GPS速度健康狀態
    bool odom_healthy_ = false;     // 新增里程計健康狀態

    // EKF狀態 (16維)
    // 0-2: 位置 (x, y, z)
    // 3-5: 速度 (vx, vy, vz)
    // 6-9: 四元數 (qw, qx, qy, qz)
    // 10-12: 加速度偏差 (bax, bay, baz)
    // 13-15: 角速度偏差 (bwx, bwy, bwz)
    Eigen::VectorXd x_;
    
    // 狀態協方差矩陣
    Eigen::MatrixXd P_;
    
    // 過程噪聲協方差矩陣
    Eigen::MatrixXd Q_;
    
    // GPS原點 (緯度, 經度, 高度)
    double gps_origin_lat_;
    double gps_origin_lon_;
    double gps_origin_alt_;
    bool gps_origin_set_;
    
    // 地球半徑 (m)
    const double EARTH_RADIUS = 6378137.0;
    
    // 重力加速度 (m/s^2)
    const double GRAVITY = 9.80665;
    
    // 初始化數據記錄
    void initDataLogging() {
        log_file_.open(log_file_path_);
        if (log_file_.is_open()) {
            // 寫入CSV頭部
            log_file_ << "timestamp,elapsed_time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,"
                      << "qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,"
                      << "accel_bias_x,accel_bias_y,accel_bias_z,"
                      << "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
                      << "raw_accel_x,raw_accel_y,raw_accel_z,"
                      << "raw_gyro_x,raw_gyro_y,raw_gyro_z" << std::endl;
                      
            log_start_time_ = ros::Time::now();
            ROS_INFO("EKF data logging initialized to file: %s", log_file_path_.c_str());
        } else {
            // 嘗試使用絕對路徑
            std::string home_dir = getenv("HOME") ? getenv("HOME") : ".";
            std::string alt_path = home_dir + "/ekf_log.csv";
            log_file_.open(alt_path);
            
            if (log_file_.is_open()) {
                log_file_path_ = alt_path;
                // 寫入CSV頭部
                log_file_ << "timestamp,elapsed_time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,"
                          << "qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,"
                          << "accel_bias_x,accel_bias_y,accel_bias_z,"
                          << "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
                          << "raw_accel_x,raw_accel_y,raw_accel_z,"
                          << "raw_gyro_x,raw_gyro_y,raw_gyro_z" << std::endl;
                          
                log_start_time_ = ros::Time::now();
                ROS_INFO("EKF data logging initialized to alternative path: %s", log_file_path_.c_str());
            } else {
                ROS_ERROR("Failed to open log file at both paths: %s and %s", 
                          log_file_path_.c_str(), alt_path.c_str());
                log_data_ = false;
            }
        }
    }
    
    // 初始化EKF狀態
    void initializeState() {
        // 初始化狀態向量 (16維)
        x_ = Eigen::VectorXd::Zero(16);
        
        // 初始化姿態為單位四元數
        x_(6) = 1.0; // qw
        x_(7) = 0.0; // qx
        x_(8) = 0.0; // qy
        x_(9) = 0.0; // qz
        
        // 初始化協方差矩陣
        P_ = Eigen::MatrixXd::Identity(16, 16);
        
        // 根據PX4的經驗值設置更合理的初始協方差
        // 設置位置協方差
        P_.block<3, 3>(0, 0) *= 25.0; // 增加初始位置不確定性
        
        // 設置速度協方差
        P_.block<3, 3>(3, 3) *= 5.0; // 增加速度不確定性
        
        // 設置姿態協方差
        P_.block<4, 4>(6, 6) *= 0.5; // 降低姿態不確定性
        
        // 設置加速度偏差協方差
        P_.block<3, 3>(10, 10) *= 0.1; // 調整為PX4標準值
        
        // 設置角速度偏差協方差
        P_.block<3, 3>(13, 13) *= 0.1; // 調整為PX4標準值
        
        // 初始化過程噪聲協方差矩陣 Q - 根據PX4經驗值調整
        Q_ = Eigen::MatrixXd::Identity(16, 16) * 0.01; // 全局低噪聲基線
        
        // 設置位置過程噪聲 - 位置本身不應有太大噪聲
        Q_.block<3, 3>(0, 0) *= 0.05;
        
        // 設置速度過程噪聲 - 速度變化較大，噪聲較高
        Q_.block<3, 3>(3, 3) *= 2.0;
        
        // 設置姿態過程噪聲 - 四元數噪聲應較小
        Q_.block<4, 4>(6, 6) *= 0.1;
        
        // 設置加速度偏差過程噪聲 - 緩慢變化
        Q_.block<3, 3>(10, 10) *= 0.005;
        
        // 設置角速度偏差過程噪聲 - 緩慢變化
        Q_.block<3, 3>(13, 13) *= 0.005;
        
        // 重置GPS原點
        gps_origin_set_ = false;
        
        // 重置傳感器時間戳
        last_imu_time_ = ros::Time(0);
        last_gps_time_ = ros::Time(0);
        last_mag_time_ = ros::Time(0);
        last_baro_time_ = ros::Time(0);
        
        // 初始化傳感器健康狀態
        imu_healthy_ = false;
        gps_healthy_ = false;
        mag_healthy_ = false;
        baro_healthy_ = false;
    }
    
    // 傳感器回調函數
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        latest_imu_ = *msg;
        last_imu_time_ = msg->header.stamp;
    }
    
    // GPS回調函數 - 使用原始GPS數據
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // 檢查GPS數據有效性
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(5.0, "No GPS fix, skipping data");
            gps_healthy_ = false;
            return;
        }
        
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude) ||
            std::isinf(msg->latitude) || std::isinf(msg->longitude) || std::isinf(msg->altitude)) {
            ROS_WARN_THROTTLE(5.0, "Invalid GPS data received, skipping");
            gps_healthy_ = false;
            return;
        }
        
        // 檢查衛星數量是否足夠（僅當有衛星數據時）
        if (latest_gps_satellites_ < min_gps_satellites_) {
            ROS_WARN_THROTTLE(5.0, "Insufficient GPS satellites (%d/%d), skipping data", 
                            latest_gps_satellites_, min_gps_satellites_);
            return;
        }
        
        latest_gps_ = *msg;
        last_gps_time_ = msg->header.stamp;
        
        // 優化協方差使用 - 根據position_covariance_type調整
        if (msg->position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            // 直接使用來自GPS的協方差（如果有效）
            ROS_DEBUG("Using GPS provided covariance, type: %d", msg->position_covariance_type);
        } else {
            // 如果協方差未知，設定默認值
            ROS_DEBUG_THROTTLE(10.0, "GPS covariance unknown, using defaults");
        }
        
        // 設置GPS原點 (如果尚未設置)
        if (!gps_origin_set_ && msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            gps_origin_lat_ = msg->latitude;
            gps_origin_lon_ = msg->longitude;
            gps_origin_alt_ = msg->altitude;
            gps_origin_set_ = true;
            
            // 重置EKF位置到原點
            x_.segment<3>(0).setZero();
            x_.segment<3>(3).setZero(); // 初始速度也設為零
            
            ROS_INFO("GPS origin set: Latitude=%.8f, Longitude=%.8f, Altitude=%.2f", 
                     gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
        }
        
        // 標記GPS狀態為健康
        gps_healthy_ = true;
    }
    
    // 新增：GPS速度回調函數
    void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 檢查數據有效性
        if (std::isnan(msg->twist.linear.x) || std::isnan(msg->twist.linear.y) || std::isnan(msg->twist.linear.z) ||
            std::isinf(msg->twist.linear.x) || std::isinf(msg->twist.linear.y) || std::isinf(msg->twist.linear.z)) {
            ROS_WARN_THROTTLE(5.0, "Invalid GPS velocity data received, skipping");
            gps_vel_healthy_ = false;
            return;
        }
        
        latest_gps_vel_ = *msg;
        last_gps_vel_time_ = msg->header.stamp;
        gps_vel_healthy_ = true;
    }
    
    // 新增：GPS衛星數量回調函數
    void gpsSatCallback(const std_msgs::UInt32::ConstPtr& msg) {
        latest_gps_satellites_ = static_cast<int>(msg->data);
        ROS_DEBUG_THROTTLE(5.0, "GPS satellites in view: %d", latest_gps_satellites_);
    }
    
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        latest_mag_ = *msg;
        last_mag_time_ = msg->header.stamp;
    }
    
    void baroCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
        latest_baro_ = *msg;
        // 使用消息中的時間戳，而不是當前時間
        last_baro_time_ = msg->header.stamp;
        
        // 只有在數據有效時才設置健康狀態為真
        if (!std::isnan(msg->fluid_pressure) && !std::isinf(msg->fluid_pressure)) {
            baro_healthy_ = true;
        }
    }
    
    // 新增：里程計回調函數
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 首先檢查是否啟用里程計，如果未啟用則直接返回
        if (!use_odom_) {
            return;  // 如果未啟用里程計，直接返回不處理數據
        }
        
        // 檢查數據有效性
        if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y) || 
            std::isnan(msg->pose.pose.position.z) || std::isnan(msg->pose.pose.orientation.w) || 
            std::isnan(msg->pose.pose.orientation.x) || std::isnan(msg->pose.pose.orientation.y) || 
            std::isnan(msg->pose.pose.orientation.z)) {
            ROS_WARN_THROTTLE(5.0, "Invalid odometry data received, skipping");
            odom_healthy_ = false;
            return;
        }
        
        // 儲存里程計數據
        latest_odom_ = *msg;
        last_odom_time_ = msg->header.stamp;
        
        // 標記里程計狀態為健康
        odom_healthy_ = true;
        
        // Debug輸出 - 每50條消息輸出一次
        static int odom_count = 0;
        if (++odom_count % 50 == 0) {
            ROS_DEBUG("接收到里程計數據 #%d: x=%.2f, y=%.2f, z=%.2f", 
                     odom_count, 
                     msg->pose.pose.position.x, 
                     msg->pose.pose.position.y, 
                     msg->pose.pose.position.z);
        }
    }
    
    // EKF更新回調函數
    void updateCallback(const ros::TimerEvent& event) {
        ros::Time current_time = ros::Time::now();
        
        // 檢查傳感器健康狀態
        checkSensorHealth(current_time);
        
        // 檢查是否已收到IMU數據
        if (!imu_healthy_) {
            ROS_WARN_THROTTLE(1.0, "IMU data not healthy, cannot perform EKF update");
            return;
        }
        
        // 進行EKF預測步驟 (使用IMU數據)
        predict(latest_imu_, current_time);
        
        // 檢查並應用GPS測量更新，使用傳感器健康度和衛星數量來判斷
        if (use_gps_ && gps_origin_set_ && gps_healthy_ && latest_gps_satellites_ >= min_gps_satellites_) {
            updateGPS(latest_gps_);
            
            // 如果啟用GPS速度並且數據健康，更新速度
            if (use_gps_vel_ && gps_vel_healthy_) {
                updateGPSVelocity(latest_gps_vel_);
            }
        }
        
        // 檢查並應用里程計測量更新，使用傳感器健康度來判斷
        // 必須同時滿足：1.啟用里程計功能 2.里程計數據健康
        if (use_odom_ && odom_healthy_) {
            ROS_DEBUG_THROTTLE(2.0, "Updating with odometry data");
            updateOdometry(latest_odom_);
        }
        
        // 檢查並應用磁力計測量更新，使用傳感器健康度來判斷
        if (use_mag_ && mag_healthy_) {
            updateMag(latest_mag_);
        }
        
        // 檢查並應用氣壓計測量更新，使用傳感器健康度來判斷
        if (use_baro_ && baro_healthy_) {
            updateBaro(latest_baro_);
        }
        
        // 發布最新的估計結果
        publishEstimates(current_time);
    }
    
    // EKF預測步驟
    void predict(const sensor_msgs::Imu& imu, const ros::Time& time) {
        // 獲取IMU測量
        Eigen::Vector3d acc(imu.linear_acceleration.x, 
                           imu.linear_acceleration.y, 
                           imu.linear_acceleration.z);
        
        Eigen::Vector3d gyro(imu.angular_velocity.x, 
                            imu.angular_velocity.y, 
                            imu.angular_velocity.z);
        
        // 計算時間步長
        static ros::Time last_predict_time = time;
        double dt = (time - last_predict_time).toSec();
        last_predict_time = time;
        
        // 限制時間步長，防止過大的跳躍
        if (dt > 0.1) dt = 0.1;
        if (dt <= 0.0) return;
        
        // 獲取當前狀態
        Eigen::Vector3d pos = x_.segment<3>(0);
        Eigen::Vector3d vel = x_.segment<3>(3);
        Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
        Eigen::Vector3d acc_bias = x_.segment<3>(10);
        Eigen::Vector3d gyro_bias = x_.segment<3>(13);
        
        // 移除加速度偏差
        Eigen::Vector3d acc_unbiased = acc - acc_bias;
        
        // 移除角速度偏差
        Eigen::Vector3d gyro_unbiased = gyro - gyro_bias;
        
        // 將機體坐標系的加速度轉到慣性坐標系（NED框架）
        Eigen::Vector3d acc_inertial = q._transformVector(acc_unbiased);
        
        // 在NED坐標系中，重力沿Z軸正方向，所以減去重力
        acc_inertial(2) -= GRAVITY;  // 正確的重力補償方向
        
        // 改進的靜止檢測 - 基於加速度和角速度的綜合門限
        double acc_norm = acc.norm();
        double gyro_norm = gyro_unbiased.norm();
        
        // 使用更寬鬆的加速度範圍和更合理的角速度閾值
        bool acc_stationary = (acc_norm > 9.0 && acc_norm < 10.5);
        bool gyro_stationary = (gyro_norm < 0.1); // 提高角速度閾值
        
        // 綜合判斷靜止狀態 - 需要同時滿足加速度和角速度條件
        bool is_stationary = acc_stationary && gyro_stationary;
        
        // 使用靜止檢測結果調整過程噪聲（動態調整）
        if (is_stationary) {
            // 靜止時減小過程噪聲
            Q_.block<3, 3>(3, 3) *= 0.1; // 降低速度噪聲
            Q_.block<4, 4>(6, 6) *= 0.1; // 降低姿態噪聲
        } else {
            // 運動時恢復正常過程噪聲
            Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.2;
            Q_.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() * 0.01;
        }
        
        // 在靜止狀態下，強制速度為零，避免漂移
        // 修改：註解掉強制歸零的邏輯，以觀察純IMU漂移
        
        if (is_stationary) {
            acc_inertial.setZero();
            vel.setZero();
        }
        
        
        
        
        // 更新位置 (半積分)
        pos += vel * dt + 0.5 * acc_inertial * dt * dt;
        
        // 更新速度
        vel += acc_inertial * dt;
        
        // 在靜止狀態下，強制速度為零，避免漂移
        // 修改：註解掉強制歸零的邏輯
        
        if (is_stationary) {
            vel.setZero();
        }
        
        
        
        // 更新姿態 (四元數積分)
        Eigen::Quaterniond dq;
        gyro_norm = gyro_unbiased.norm();
        
        if (gyro_norm > 1e-10) {
            double angle = gyro_norm * dt * 0.5;
            Eigen::Vector3d axis = gyro_unbiased / gyro_norm;
            dq = Eigen::Quaterniond(std::cos(angle), 
                                   axis.x() * std::sin(angle),
                                   axis.y() * std::sin(angle),
                                   axis.z() * std::sin(angle));
        } else {
            dq = Eigen::Quaterniond(1, 0, 0, 0);
        }
        
        q = q * dq;
        q.normalize(); // 確保四元數規範化
        
        // 更新狀態
        x_.segment<3>(0) = pos;
        x_.segment<3>(3) = vel;
        x_(6) = q.w();
        x_(7) = q.x();
        x_(8) = q.y();
        x_(9) = q.z();
        
        // 更新狀態協方差矩陣 - 使用標準EKF方程
        // 計算狀態轉移雅可比矩陣 F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(16, 16);
        
        // 位置對速度的雅可比
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        
        // 角速度轉換為四元數變化率的雅可比（簡化版本）
        if (gyro_norm > 1e-10) {
            Eigen::Vector3d axis = gyro_unbiased / gyro_norm;
            double angle = gyro_norm * dt * 0.5;
            double sinAngle = std::sin(angle);
            
            // 簡化的四元數雅可比
            F.block<4, 3>(6, 13) = Eigen::MatrixXd::Zero(4, 3);
            F(6, 13) = -0.5 * sinAngle * axis.x() * dt;
            F(6, 14) = -0.5 * sinAngle * axis.y() * dt;
            F(6, 15) = -0.5 * sinAngle * axis.z() * dt;
            F(7, 13) = 0.5 * std::cos(angle) * dt;
            F(8, 14) = 0.5 * std::cos(angle) * dt;
            F(9, 15) = 0.5 * std::cos(angle) * dt;
        }
        
        // 更新協方差 P = F*P*F' + Q
        P_ = F * P_ * F.transpose() + Q_ * dt;
        
        // 確保協方差矩陣對稱正定
        P_ = 0.5 * (P_ + P_.transpose());
        
        // 處理數值誤差，確保協方差對角元素不爲負
        for (int i = 0; i < 16; i++) {
            if (P_(i, i) < 0.0) {
                P_(i, i) = 1e-6;
            }
        }
    }
    
    // GPS測量更新 - 使用原始GPS數據的改進版本
    void updateGPS(const sensor_msgs::NavSatFix& gps) {
        // 檢查GPS數據有效性
        if (gps.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN_THROTTLE(5.0, "No GPS fix, skipping data");
            return;
        }
        
        if (std::isnan(gps.latitude) || std::isnan(gps.longitude) || std::isnan(gps.altitude) ||
            std::isinf(gps.latitude) || std::isinf(gps.longitude) || std::isinf(gps.altitude)) {
            ROS_WARN_THROTTLE(5.0, "Invalid GPS data received, skipping");
            return;
        }
        
        // 設置GPS原點 (如果尚未設置)
        if (!gps_origin_set_ && gps.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
            gps_origin_lat_ = gps.latitude;
            gps_origin_lon_ = gps.longitude;
            gps_origin_alt_ = gps.altitude;
            gps_origin_set_ = true;
            
            // 重置EKF位置到原點
            x_.segment<3>(0).setZero();
            x_.segment<3>(3).setZero(); // 初始速度也設為零
            
            ROS_INFO("GPS origin set: Latitude=%.8f, Longitude=%.8f, Altitude=%.2f", 
                     gps_origin_lat_, gps_origin_lon_, gps_origin_alt_);
            return;
        }
        
        // 將GPS坐標轉換為NED坐標
        Eigen::Vector3d ned_position = gpsToNED(gps.latitude, gps.longitude, gps.altitude);
        
        // 構建測量向量
        Eigen::VectorXd z(3);
        z << ned_position;
        
        // 構建測量矩陣 H (3x16)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 16);
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // 只觀測位置
        
        // 構建測量噪聲協方差矩陣 R
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
        
        // 處理協方差，根據GPS數據質量調整
        if (gps.position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            // 使用GPS提供的協方差，並考慮坐標轉換
            double lat_m_per_rad = EARTH_RADIUS; // 每弧度的米數（緯度）
            double lon_m_per_rad = EARTH_RADIUS * std::cos(gps.latitude * M_PI / 180.0); // 每弧度的米數（經度）
            
            // 轉換協方差從度到米
            double deg_to_rad = M_PI / 180.0;
            double lat_var_m = gps.position_covariance[0] * (lat_m_per_rad * deg_to_rad) * (lat_m_per_rad * deg_to_rad);
            double lon_var_m = gps.position_covariance[4] * (lon_m_per_rad * deg_to_rad) * (lon_m_per_rad * deg_to_rad);
            double alt_var_m = gps.position_covariance[8];
            
            // 調整協方差，確保數值穩定
            R(0, 0) = std::max(lat_var_m, gps_noise_lat_lon_); // 北向
            R(1, 1) = std::max(lon_var_m, gps_noise_lat_lon_); // 東向
            R(2, 2) = std::max(alt_var_m, gps_noise_alt_);    // 下向（高度通常更不准確）
        } else {
            // 使用默認噪聲參數
            R(0, 0) = gps_noise_lat_lon_;  // 北向
            R(1, 1) = gps_noise_lat_lon_;  // 東向
            R(2, 2) = gps_noise_alt_;     // 下向
        }
        
        // 基於衛星數量動態調整噪聲
        if (latest_gps_satellites_ > 0) {
            double sat_factor = std::min(1.0, std::max(0.5, (double)latest_gps_satellites_ / 12.0));
            R *= (2.0 - sat_factor);  // 衛星數量越多，噪聲越小
        }
        
        // 進行卡爾曼增益計算
        Eigen::MatrixXd PHt = P_ * H.transpose();
        Eigen::MatrixXd S = H * PHt + R;
        Eigen::MatrixXd K = PHt * S.inverse();
        
        // 計算測量預測
        Eigen::VectorXd z_pred(3);
        z_pred = H * x_;
        
        // 計算新息
        Eigen::VectorXd y = z - z_pred;
        
        // 檢測異常值 - 基於配置的創新閾值
        double innovation_threshold = gps_pos_innov_gate_;
        bool large_innovation = false;
        
        for (int i = 0; i < 3; i++) {
            if (std::abs(y(i)) > innovation_threshold * std::sqrt(S(i,i))) {
                ROS_WARN("GPS measurement %d innovation too large: %.2f m (threshold: %.2f). Limiting.", 
                         i, y(i), innovation_threshold * std::sqrt(S(i,i)));
                y(i) = (y(i) > 0) ? innovation_threshold * std::sqrt(S(i,i)) : -innovation_threshold * std::sqrt(S(i,i));
                large_innovation = true;
            }
        }
        
        // 如果有大創新，調整卡爾曼增益
        if (large_innovation) {
            K *= 0.5;  // 減小增益以減少異常值的影響
        }
        
        // 更新狀態 - 使用Joseph形式，提高數值穩定性
        x_ = x_ + K * y;
        
        // 正規化四元數
        double qw = x_(6);
        double qx = x_(7);
        double qy = x_(8);
        double qz = x_(9);
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        
        x_(6) = qw / q_norm;
        x_(7) = qx / q_norm;
        x_(8) = qy / q_norm;
        x_(9) = qz / q_norm;
        
        // 更新協方差 - 使用Joseph形式，提高數值穩定性
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(16, 16) - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        
        // 確保協方差矩陣對稱
        P_ = 0.5 * (P_ + P_.transpose());
        
        // 檢查位置新息，用於診斷
        ROS_DEBUG("GPS position innovation: N=%.2f, E=%.2f, D=%.2f (m)", y(0), y(1), y(2));
    }
    
    // 新增：GPS速度測量更新
    void updateGPSVelocity(const geometry_msgs::TwistStamped& gps_vel) {
        // 檢查速度數據有效性
        if (std::isnan(gps_vel.twist.linear.x) || std::isnan(gps_vel.twist.linear.y) || std::isnan(gps_vel.twist.linear.z) ||
            std::isinf(gps_vel.twist.linear.x) || std::isinf(gps_vel.twist.linear.y) || std::isinf(gps_vel.twist.linear.z)) {
            ROS_WARN_THROTTLE(5.0, "Invalid GPS velocity data, skipping velocity update");
            return;
        }
        
        // 獲取當前估計的姿態四元數，用於坐標轉換
        Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
        
        // 構建測量向量 (NED坐標系中的速度)
        Eigen::VectorXd z(3);
        
        // 假設GPS速度已經是在地球坐標系 (ENU)，轉換到NED坐標系
        double vel_n = gps_vel.twist.linear.y;  // ENU中的y對應NED中的North
        double vel_e = gps_vel.twist.linear.x;  // ENU中的x對應NED中的East
        double vel_d = -gps_vel.twist.linear.z; // ENU中的z向上，NED中的D向下，需要取反
        
        z << vel_n, vel_e, vel_d;
        
        // 構建測量矩陣 H (3x16)，連接狀態變量中的速度
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 16);
        H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // 觀測速度 (狀態向量的索引3-5)
        
        // 構建測量噪聲協方差矩陣 R
        double vel_noise = 0.1; // 速度測量噪聲 (m/s)
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * vel_noise * vel_noise;
        
        // 基於衛星數量動態調整噪聲
        if (latest_gps_satellites_ > 0) {
            double sat_factor = std::min(1.0, std::max(0.5, (double)latest_gps_satellites_ / 12.0));
            R *= (2.0 - sat_factor);  // 衛星數量越多，噪聲越小
        }
        
        // 進行卡爾曼增益計算
        Eigen::MatrixXd PHt = P_ * H.transpose();
        Eigen::MatrixXd S = H * PHt + R;
        Eigen::MatrixXd K = PHt * S.inverse();
        
        // 計算測量預測 (當前估計的速度)
        Eigen::Vector3d vel_pred = x_.segment<3>(3); // 狀態向量中的速度部分
        
        // 計算新息 (測量減去預測)
        Eigen::VectorXd y = z - vel_pred;
        
        // 檢測異常值
        double vel_innov_gate = 5.0; // 速度創新閾值
        bool large_innovation = false;
        
        for (int i = 0; i < 3; i++) {
            if (std::abs(y(i)) > vel_innov_gate) {
                ROS_WARN("GPS velocity innovation %d too large: %.2f m/s. Limiting.", i, y(i));
                y(i) = (y(i) > 0) ? vel_innov_gate : -vel_innov_gate;
                large_innovation = true;
            }
        }
        
        // 如果有大創新，調整卡爾曼增益
        if (large_innovation) {
            K *= 0.5;  // 減小增益以減少異常值的影響
        }
        
        // 更新狀態
        x_ = x_ + K * y;
        
        // 正規化四元數
        double qw = x_(6);
        double qx = x_(7);
        double qy = x_(8);
        double qz = x_(9);
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        
        x_(6) = qw / q_norm;
        x_(7) = qx / q_norm;
        x_(8) = qy / q_norm;
        x_(9) = qz / q_norm;
        
        // 更新協方差
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(16, 16) - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        
        // 確保協方差矩陣對稱
        P_ = 0.5 * (P_ + P_.transpose());
        
        // 檢查速度新息，用於診斷
        ROS_DEBUG("GPS velocity innovation: N=%.2f, E=%.2f, D=%.2f (m/s)", y(0), y(1), y(2));
    }
    
    // 磁力計測量更新 - 改進版本
    void updateMag(const sensor_msgs::MagneticField& mag) {
        // 獲取磁力計測量
        Eigen::Vector3d mag_meas(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
        
        // 檢查磁力計數據有效性
        double mag_norm = mag_meas.norm();
        if (mag_norm < 1e-10 || std::isnan(mag_norm) || std::isinf(mag_norm)) {
            ROS_WARN_THROTTLE(5.0, "Invalid magnetometer data received, skipping update");
            return;
        }
        

        // 獲取當前估計的姿態四元數
        Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
        
        // 從四元數中提取歐拉角
        Eigen::Vector3d euler = quaternionToEuler(q);
        double roll = euler(0);
        double pitch = euler(1);
        double yaw = euler(2);
        
        // 構建旋轉矩陣，將磁場從機體坐標系轉到水平儀表坐標系
        Eigen::Quaterniond q_roll_pitch = eulerToQuaternion(roll, pitch, 0.0);
        Eigen::Vector3d mag_flat = q_roll_pitch.inverse()._transformVector(mag_meas);
        
        // 計算測量的航向角
        double measured_heading = std::atan2(mag_flat(1), mag_flat(0));
        
        // 計算航向角的新息
        double heading_innovation = measured_heading - yaw;
        
        // 標準化到 [-pi, pi]
        while (heading_innovation > M_PI) heading_innovation -= 2.0 * M_PI;
        while (heading_innovation < -M_PI) heading_innovation += 2.0 * M_PI;
        
        // 計算磁力計測量噪聲的自適應增益
        // 根據磁場強度和傾斜角度動態調整
        double tilt_angle = std::sqrt(roll*roll + pitch*pitch);
        double mag_stability = 1.0 / (1.0 + 2.0 * tilt_angle);  // 傾斜越大，穩定性越差
        
        // 根據磁力計噪聲參數和穩定性計算增益
        double heading_gain = mag_noise_ * mag_stability;
        
        // 限制增益範圍
        if (heading_gain < 0.01) heading_gain = 0.01;
        if (heading_gain > 0.5) heading_gain = 0.5;
        
        // 更新航向角
        yaw += heading_gain * heading_innovation;
        
        // 標準化到 [-pi, pi]
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        
        // 從更新後的歐拉角重建四元數
        Eigen::Quaterniond updated_q = eulerToQuaternion(roll, pitch, yaw);
        
        // 更新狀態向量中的四元數
        x_(6) = updated_q.w();
        x_(7) = updated_q.x();
        x_(8) = updated_q.y();
        x_(9) = updated_q.z();
        
        // 更新四元數相關的協方差
        // 簡化起見，我們只減少與航向相關的不確定性
        P_(6, 6) *= (1.0 - heading_gain * 0.5);
        P_(7, 7) *= (1.0 - heading_gain * 0.5);
        P_(8, 8) *= (1.0 - heading_gain * 0.5);
        P_(9, 9) *= (1.0 - heading_gain * 0.5);
    }
    
    // 氣壓計測量更新 - 改進版本
    void updateBaro(const sensor_msgs::FluidPressure& baro) {
        // 獲取氣壓計測量值
        double pressure = baro.fluid_pressure;
        
        // 檢查數據有效性
        if (std::isnan(pressure) || std::isinf(pressure)) {
            ROS_WARN_THROTTLE(5.0, "Invalid barometer data received, skipping update");
            return;
        }
        
        // 將壓力轉換為高度 (簡化的氣壓高度公式)
        // P = P0 * exp(-height / scale_height)
        // 解出 height = -scale_height * ln(P/P0)
        const double P0 = 101325.0;  // 標準海平面氣壓 (Pa)
        const double scale_height = 8400.0;  // 近似壓力尺度高度 (m)
        double measured_height = -scale_height * std::log(pressure / P0);
        
        // 初始化參考高度（如果尚未初始化）
        static double baro_ref_height = measured_height;
        static bool baro_ref_initialized = false;
        static ros::Time last_baro_ref_time = ros::Time::now();
        
        // 定期更新氣壓計參考高度，補償溫度漂移和大氣壓變化
        ros::Time current_time = ros::Time::now();
        double time_since_last_ref = (current_time - last_baro_ref_time).toSec();
        
        if (!baro_ref_initialized) {
            baro_ref_height = measured_height;
            baro_ref_initialized = true;
            last_baro_ref_time = current_time;
            ROS_INFO("Initialized barometer reference height to %.2f m", baro_ref_height);
            return;
        } else if (time_since_last_ref > static_pressure_timeout_) {
            // 每隔一定時間重新校準氣壓計參考高度，使用當前估計高度
            double current_est_height = -x_(2);  // NED系統中，高度是-Z
            double height_diff = measured_height - baro_ref_height;
            
            // 更新參考高度，但限制變化率
            double max_change = 0.5;  // 每次最大變化0.5米
            double change = std::min(std::max(height_diff - current_est_height, -max_change), max_change);
            baro_ref_height += change;
            
            last_baro_ref_time = current_time;
            ROS_INFO("Updated barometer reference height to %.2f m", baro_ref_height);
        }
        
        // 計算相對於參考高度的測量值
        double relative_measured_height = measured_height - baro_ref_height;
        
        // 當前估計的高度（在NED坐標系中，Z軸向下為正，高度為-Z）
        double estimated_height = -x_(2);
        
        // 計算高度創新（測量值與估計值之差）
        double height_innovation = relative_measured_height - estimated_height;
        
        // 動態調整氣壓計更新的增益
        // 根據當前高度和時間自適應調整
        double height_gain = baro_noise_ / (baro_noise_ + P_(2, 2));
        height_gain = std::min(std::max(height_gain, 0.01), 0.3);  // 限制增益範圍
        
        // 更新高度狀態（在NED中，高度是-Z）
        x_(2) -= height_gain * height_innovation;
        
        // 當高度變化大時，部分更新垂直速度
        if (std::abs(height_innovation) > 0.2) {
            double vel_z_gain = 0.1 * height_gain;
            x_(5) -= vel_z_gain * height_innovation / 0.1;  // 假設0.1秒內完成變化
        }
        
        // 更新協方差
        P_(2, 2) *= (1.0 - height_gain);
        P_(5, 5) *= (1.0 - 0.1 * height_gain);
    }
    
    // 將GPS坐標轉換為NED坐標
    Eigen::Vector3d gpsToNED(double lat, double lon, double alt) {
        if (!gps_origin_set_) {
            // 如果原點未設置，返回零向量
            return Eigen::Vector3d::Zero();
        }
        
        // 轉換為弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double origin_lat_rad = gps_origin_lat_ * M_PI / 180.0;
        double origin_lon_rad = gps_origin_lon_ * M_PI / 180.0;
        
        // 計算地球表面的弧長
        double d_lat = lat_rad - origin_lat_rad;
        double d_lon = lon_rad - origin_lon_rad;
        
        // 計算北東方向的距離 (考慮地球半徑和緯度)
        double north = EARTH_RADIUS * d_lat;
        double east = EARTH_RADIUS * d_lon * std::cos(origin_lat_rad);
        
        // 計算高度差 - 在NED系統中，向下為正
        double down = gps_origin_alt_ - alt;  // 修正：NED系統中高度差，原點高度減去測量高度
        
        // 限制異常值 - 如果高度差過大，可能是錯誤的測量
        if (std::abs(down) > 1000.0) {
            ROS_WARN("Large altitude difference detected: %.2f m. Limiting to ±50m", down);
            down = (down > 0) ? 50.0 : -50.0;
        }
        
        // 返回NED坐標 (北、東、下)
        return Eigen::Vector3d(north, east, down);
    }
    
    // 四元數轉歐拉角
    Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
        // 從四元數中提取歐拉角 (ZYX順序)
        double roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                               1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        
        double pitch = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        
        double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        
        return Eigen::Vector3d(roll, pitch, yaw);
    }
    
    // 歐拉角轉四元數
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
        // 創建旋轉矩陣 (ZYX順序)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        return q;
    }
    
    // Publish latest EKF estimation results
    void publishEstimates(const ros::Time& time) {
        // Get current state
        Eigen::Vector3d pos = x_.segment<3>(0);
        Eigen::Vector3d vel = x_.segment<3>(3);
        Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
        Eigen::Vector3d accel_bias = x_.segment<3>(10);
        Eigen::Vector3d gyro_bias = x_.segment<3>(13);
        
        // Get Euler angles
        Eigen::Vector3d euler = quaternionToEuler(q);
        double roll = euler(0) * 180.0 / M_PI;
        double pitch = euler(1) * 180.0 / M_PI;
        double yaw = euler(2) * 180.0 / M_PI;
        
        // Log data to CSV file
        if (log_data_ && log_file_.is_open()) {
            // Get raw IMU data
            Eigen::Vector3d raw_accel(
                latest_imu_.linear_acceleration.x,
                latest_imu_.linear_acceleration.y,
                latest_imu_.linear_acceleration.z
            );
            
            Eigen::Vector3d raw_gyro(
                latest_imu_.angular_velocity.x,
                latest_imu_.angular_velocity.y,
                latest_imu_.angular_velocity.z
            );
            
            // Calculate elapsed time since recording started
            double elapsed_time = (time - log_start_time_).toSec();
            
            // Write CSV line
            log_file_ << std::fixed << std::setprecision(6)
                     << time.toSec() << "," 
                     << elapsed_time << "," 
                     << pos(0) << "," << pos(1) << "," << pos(2) << ","
                     << vel(0) << "," << vel(1) << "," << vel(2) << ","
                     << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
                     << roll << "," << pitch << "," << yaw << ","
                     << accel_bias(0) << "," << accel_bias(1) << "," << accel_bias(2) << ","
                     << gyro_bias(0) << "," << gyro_bias(1) << "," << gyro_bias(2) << ","
                     << raw_accel(0) << "," << raw_accel(1) << "," << raw_accel(2) << ","
                     << raw_gyro(0) << "," << raw_gyro(1) << "," << raw_gyro(2)
                     << std::endl;
        }
        
        // Publish pose estimation
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.header.frame_id = map_frame_id_;
        
        // Set position
        pose_msg.pose.pose.position.x = pos(0);
        pose_msg.pose.pose.position.y = pos(1);
        pose_msg.pose.pose.position.z = pos(2);
        
        // Set orientation
        pose_msg.pose.pose.orientation.w = q.w();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        
        // Set covariance (position and orientation only)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                // Position covariance
                pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
                // Orientation covariance (simplified)
                pose_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = P_(i + 6, j + 6);
            }
        }
        
        pose_pub_.publish(pose_msg);
        
        // Publish odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = map_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        
        // Set position and orientation
        odom_msg.pose.pose.position.x = pos(0);
        odom_msg.pose.pose.position.y = pos(1);
        odom_msg.pose.pose.position.z = pos(2);
        
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        
        // Set covariance
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                // Position covariance
                odom_msg.pose.covariance[i * 6 + j] = P_(i, j);
                // Orientation covariance (simplified)
                odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = P_(i + 6, j + 6);
            }
        }
        
        // Set velocity
        odom_msg.twist.twist.linear.x = vel(0);
        odom_msg.twist.twist.linear.y = vel(1);
        odom_msg.twist.twist.linear.z = vel(2);
        
        // Calculate unbiased angular velocity from quaternion rate (simplified)
        Eigen::Vector3d gyro(latest_imu_.angular_velocity.x, 
                            latest_imu_.angular_velocity.y, 
                            latest_imu_.angular_velocity.z);
        Eigen::Vector3d gyro_unbiased = gyro - gyro_bias;
        
        odom_msg.twist.twist.angular.x = gyro_unbiased(0);
        odom_msg.twist.twist.angular.y = gyro_unbiased(1);
        odom_msg.twist.twist.angular.z = gyro_unbiased(2);
        
        // Set velocity covariance
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                // Linear velocity covariance
                odom_msg.twist.covariance[i * 6 + j] = P_(i + 3, j + 3);
                // Angular velocity covariance (simplified)
                odom_msg.twist.covariance[(i + 3) * 6 + (j + 3)] = 0.01;
            }
        }
        
        odom_pub_.publish(odom_msg);
        
        // Publish velocity
        geometry_msgs::TwistStamped velocity_msg;
        velocity_msg.header.stamp = time;
        velocity_msg.header.frame_id = map_frame_id_;
        
        // Set linear velocity
        velocity_msg.twist.linear.x = vel(0);
        velocity_msg.twist.linear.y = vel(1);
        velocity_msg.twist.linear.z = vel(2);
        
        // Set angular velocity from unbiased gyro
        velocity_msg.twist.angular.x = gyro_unbiased(0);
        velocity_msg.twist.angular.y = gyro_unbiased(1);
        velocity_msg.twist.angular.z = gyro_unbiased(2);
        
        velocity_pub_.publish(velocity_msg);
        
        // Publish TF transform
        if (publish_tf_) {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = time;
            transform.header.frame_id = map_frame_id_;
            transform.child_frame_id = base_frame_id_;
            
            transform.transform.translation.x = pos(0);
            transform.transform.translation.y = pos(1);
            transform.transform.translation.z = pos(2);
            
            transform.transform.rotation.w = q.w();
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            
            tf_broadcaster_.sendTransform(transform);
        }
        
        // 顯著減少打印頻率 - 從1秒改為30秒打印一次
        static ros::Time last_print_time = ros::Time(0);
        if ((time - last_print_time).toSec() >= 30.0) {  // 原本是1.0秒
            last_print_time = time;
            
            ROS_INFO("------ EKF Basic Position Estimate (%.2f) ------", time.toSec());
            ROS_INFO("Position (m): X=%.2f Y=%.2f Z=%.2f", pos(0), pos(1), pos(2));
            ROS_INFO(" "); // Add empty line for readability
        }

        // 減少傳感器健康狀態打印頻率 - 從10秒改為60秒打印一次
        static ros::Time last_health_print = ros::Time(0);
        if ((time - last_health_print).toSec() > 60.0) {  // 原本是10.0秒
            last_health_print = time;
            ROS_INFO("Sensor Health Status - IMU: %s, GPS: %s, Vel: %s, Mag: %s, Baro: %s",
                    imu_healthy_ ? "OK" : "FAIL",
                    gps_healthy_ ? "OK" : "FAIL",
                    gps_vel_healthy_ ? "OK" : "FAIL",
                    mag_healthy_ ? "OK" : "FAIL",
                    baro_healthy_ ? "OK" : "FAIL");
        }
    }

    // 檢查傳感器數據的健康性
    void checkSensorHealth(const ros::Time& current_time) {
        // 檢查IMU數據，如果超過0.1秒沒有更新就認為不健康
        imu_healthy_ = (current_time - last_imu_time_).toSec() < 0.1;
        
        // 檢查GPS數據，如果超過1秒沒有更新就認為不健康
        gps_healthy_ = use_gps_ && ((current_time - last_gps_time_).toSec() < 1.0);
        
        // 檢查GPS衛星數量
        if (gps_healthy_ && latest_gps_satellites_ < min_gps_satellites_) {
            ROS_WARN_THROTTLE(5.0, "GPS satellite count below minimum (%d/%d)", 
                            latest_gps_satellites_, min_gps_satellites_);
            gps_healthy_ = false;
        }
        
        // 檢查GPS速度數據
        gps_vel_healthy_ = use_gps_vel_ && ((current_time - last_gps_vel_time_).toSec() < 1.0);
        
        // 檢查磁力計數據，如果超過1秒沒有更新就認為不健康
        mag_healthy_ = use_mag_ && ((current_time - last_mag_time_).toSec() < 1.0);
        
        // 檢查氣壓計數據，如果超過1秒沒有更新就認為不健康
        baro_healthy_ = use_baro_ && ((current_time - last_baro_time_).toSec() < 1.0);
        
        // 檢查里程計數據，必須同時滿足：1.啟用里程計 2.數據時間有效
        odom_healthy_ = use_odom_ && ((current_time - last_odom_time_).toSec() < 1.0);
        
        // 如果未啟用里程計，確保odom_healthy_為false
        if (!use_odom_) {
            odom_healthy_ = false;
        }
        
        // 輸出傳感器健康狀態
        static int status_count = 0;
        if (++status_count >= 50) {  // 每50次迭代顯示一次傳感器狀態
            status_count = 0;
            ROS_INFO("Sensor health - IMU: %s, GPS: %s (Sats: %d), Vel: %s, Mag: %s, Baro: %s, Odom: %s",
                     imu_healthy_ ? "OK" : "FAIL",
                     gps_healthy_ ? "OK" : "FAIL",
                     latest_gps_satellites_,
                     gps_vel_healthy_ ? "OK" : "FAIL",
                     mag_healthy_ ? "OK" : "FAIL",
                     baro_healthy_ ? "OK" : "FAIL",
                     odom_healthy_ ? "OK" : "FAIL");
        }
    }

    // 新增：里程計測量更新
    void updateOdometry(const nav_msgs::Odometry& odom) {
        // 檢查數據有效性
        if (std::isnan(odom.pose.pose.position.x) || std::isnan(odom.pose.pose.position.y) || 
            std::isnan(odom.pose.pose.position.z) || std::isnan(odom.pose.pose.orientation.w) || 
            std::isnan(odom.pose.pose.orientation.x) || std::isnan(odom.pose.pose.orientation.y) || 
            std::isnan(odom.pose.pose.orientation.z)) {
            ROS_WARN_THROTTLE(5.0, "Invalid odometry data received, skipping");
            odom_healthy_ = false;
            return;
        }
        
        // 先處理位置和姿態的更新
        // --------------------------
        
        // 從里程計中獲取位置測量
        Eigen::Vector3d odom_pos(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z
        );
        
        // 如果使用NED坐標系，需要從ENU轉換到NED
        if (use_ned_frame_) {
            // ENU到NED的轉換: (x,y,z) -> (y,x,-z)
            double tmp = odom_pos(0);
            odom_pos(0) = odom_pos(1);    // ENU的y -> NED的x (北)
            odom_pos(1) = tmp;            // ENU的x -> NED的y (東)
            odom_pos(2) = -odom_pos(2);   // ENU的z -> NED的z (下)
        }
        
        // 構建位置測量向量和測量矩陣 - 根據是否使用高度進行調整
        Eigen::VectorXd z_pos;
        Eigen::MatrixXd H_pos;
        Eigen::MatrixXd R_pos;
        
        if (use_odom_height_) {
            // 如果使用里程計高度，使用全部3個維度
            z_pos = Eigen::VectorXd(3);
            z_pos << odom_pos;
            
            // 構建位置測量矩陣 H (3x16)
            H_pos = Eigen::MatrixXd::Zero(3, 16);
            H_pos.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // 觀測位置
            
            // 構建位置測量噪聲協方差矩陣 R
            R_pos = Eigen::MatrixXd::Identity(3, 3);
            
            // 使用里程計位置噪聲參數
            R_pos(0, 0) = odom_pos_noise_ * odom_pos_noise_; // 北/X方向
            R_pos(1, 1) = odom_pos_noise_ * odom_pos_noise_; // 東/Y方向
            R_pos(2, 2) = odom_pos_noise_ * 4.0; // 下/Z方向 (通常垂直方向精度較差)
            
            // 嘗試從里程計協方差獲取更好的噪聲估計
            if (odom.pose.covariance[0] > 0 && odom.pose.covariance[7] > 0 && odom.pose.covariance[14] > 0) {
                R_pos(0, 0) = std::max(odom.pose.covariance[0], odom_pos_noise_ * odom_pos_noise_);
                R_pos(1, 1) = std::max(odom.pose.covariance[7], odom_pos_noise_ * odom_pos_noise_);
                R_pos(2, 2) = std::max(odom.pose.covariance[14], odom_pos_noise_ * 4.0);
            }
        } else {
            // 如果不使用里程計高度，僅使用水平位置 (XY)
            z_pos = Eigen::VectorXd(2);
            z_pos << odom_pos(0), odom_pos(1);
            
            // 構建位置測量矩陣 H (2x16)
            H_pos = Eigen::MatrixXd::Zero(2, 16);
            H_pos(0, 0) = 1.0; // 北/X方向
            H_pos(1, 1) = 1.0; // 東/Y方向
            
            // 構建位置測量噪聲協方差矩陣 R
            R_pos = Eigen::MatrixXd::Identity(2, 2);
            
            // 使用里程計位置噪聲參數
            R_pos(0, 0) = odom_pos_noise_ * odom_pos_noise_; // 北/X方向
            R_pos(1, 1) = odom_pos_noise_ * odom_pos_noise_; // 東/Y方向
            
            // 嘗試從里程計協方差獲取更好的噪聲估計
            if (odom.pose.covariance[0] > 0 && odom.pose.covariance[7] > 0) {
                R_pos(0, 0) = std::max(odom.pose.covariance[0], odom_pos_noise_ * odom_pos_noise_);
                R_pos(1, 1) = std::max(odom.pose.covariance[7], odom_pos_noise_ * odom_pos_noise_);
            }
            
            ROS_DEBUG("只使用XY平面里程計位置數據，忽略高度(Z軸)");
        }
        
        // 進行卡爾曼增益計算
        Eigen::MatrixXd PHt_pos = P_ * H_pos.transpose();
        Eigen::MatrixXd S_pos = H_pos * PHt_pos + R_pos;
        Eigen::MatrixXd K_pos = PHt_pos * S_pos.inverse();
        
        // 計算位置測量預測
        Eigen::VectorXd z_pos_pred;
        if (use_odom_height_) {
            z_pos_pred = Eigen::VectorXd(3);
            z_pos_pred = H_pos * x_;
        } else {
            z_pos_pred = Eigen::VectorXd(2);
            z_pos_pred << x_(0), x_(1); // 僅使用X和Y
        }
        
        // 計算位置新息
        Eigen::VectorXd y_pos = z_pos - z_pos_pred;
        
        // 檢測異常值 - 基於配置的創新閾值
        bool large_pos_innovation = false;
        
        for (int i = 0; i < y_pos.size(); i++) {
            // 使用固定閾值，而不是動態計算
            double threshold = pos_innov_gate_;
            if (std::abs(y_pos(i)) > threshold) {
                int axis = i;
                if (!use_odom_height_ && i >= 2) axis = 2; // 2D情況下映射到Z軸
                
                ROS_WARN("Odometry position %d innovation too large: %.2f m (threshold: %.2f). Limiting.", 
                         axis, y_pos(i), threshold);
                y_pos(i) = (y_pos(i) > 0) ? threshold : -threshold;
                large_pos_innovation = true;
            }
        }
        
        // 如果有大創新，調整卡爾曼增益
        if (large_pos_innovation) {
            K_pos *= 0.5;  // 減小增益以減少異常值的影響
        }
        
        // 更新狀態
        x_ += K_pos * y_pos;
        
        // 更新協方差
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(16, 16) - K_pos * H_pos;
        P_ = I_KH * P_ * I_KH.transpose() + K_pos * R_pos * K_pos.transpose();
        
        // 確保協方差矩陣對稱
        P_ = 0.5 * (P_ + P_.transpose());
        
        // 處理速度更新
        // --------------------------
        
        // 從里程計中獲取速度測量
        Eigen::Vector3d odom_vel(
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z
        );
        
        // 如果使用NED坐標系，需要從ENU轉換到NED
        if (use_ned_frame_) {
            double tmp = odom_vel(0);
            odom_vel(0) = odom_vel(1);    // ENU的y -> NED的x (北)
            odom_vel(1) = tmp;            // ENU的x -> NED的y (東)
            odom_vel(2) = -odom_vel(2);   // ENU的z -> NED的z (下)
        }
        
        // 構建速度測量向量
        Eigen::VectorXd z_vel(3);
        z_vel << odom_vel;
        
        // 構建速度測量矩陣 H (3x16)
        Eigen::MatrixXd H_vel = Eigen::MatrixXd::Zero(3, 16);
        H_vel.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // 觀測速度 (狀態向量的3-5)
        
        // 構建速度測量噪聲協方差矩陣 R
        Eigen::MatrixXd R_vel = Eigen::MatrixXd::Identity(3, 3);
        
        // 使用里程計速度噪聲參數
        R_vel(0, 0) = odom_vel_noise_ * odom_vel_noise_; // 北/X方向
        R_vel(1, 1) = odom_vel_noise_ * odom_vel_noise_; // 東/Y方向
        R_vel(2, 2) = odom_vel_noise_ * 4.0; // 下/Z方向
        
        // 嘗試從里程計協方差獲取更好的噪聲估計
        if (odom.twist.covariance[0] > 0 && odom.twist.covariance[7] > 0 && odom.twist.covariance[14] > 0) {
            // 使用里程計提供的協方差，但確保不會過小
            R_vel(0, 0) = std::max(odom.twist.covariance[0], odom_vel_noise_ * odom_vel_noise_);
            R_vel(1, 1) = std::max(odom.twist.covariance[7], odom_vel_noise_ * odom_vel_noise_);
            R_vel(2, 2) = std::max(odom.twist.covariance[14], odom_vel_noise_ * 4.0);
        }
        
        // 進行卡爾曼增益計算
        Eigen::MatrixXd PHt_vel = P_ * H_vel.transpose();
        Eigen::MatrixXd S_vel = H_vel * PHt_vel + R_vel;
        Eigen::MatrixXd K_vel = PHt_vel * S_vel.inverse();
        
        // 計算速度測量預測
        Eigen::VectorXd z_vel_pred(3);
        z_vel_pred = H_vel * x_;
        
        // 計算速度新息
        Eigen::VectorXd y_vel = z_vel - z_vel_pred;
        
        // 檢測異常值
        bool large_vel_innovation = false;
        
        for (int i = 0; i < 3; i++) {
            // 使用固定閾值，而不是動態計算
            double threshold = vel_innov_gate_;
            if (std::abs(y_vel(i)) > threshold) {
                ROS_WARN("Odometry velocity %d innovation too large: %.2f m/s (threshold: %.2f). Limiting.", 
                         i, y_vel(i), threshold);
                y_vel(i) = (y_vel(i) > 0) ? threshold : -threshold;
                large_vel_innovation = true;
            }
        }
        
        // 如果有大創新，調整卡爾曼增益
        if (large_vel_innovation) {
            K_vel *= 0.5;  // 減小增益以減少異常值的影響
        }
        
        // 更新狀態
        x_ += K_vel * y_vel;
        
        // 更新協方差
        Eigen::MatrixXd I_KH_vel = Eigen::MatrixXd::Identity(16, 16) - K_vel * H_vel;
        P_ = I_KH_vel * P_ * I_KH_vel.transpose() + K_vel * R_vel * K_vel.transpose();
        
        // 確保協方差矩陣對稱
        P_ = 0.5 * (P_ + P_.transpose());
        
        // 正規化四元數
        double qw = x_(6);
        double qx = x_(7);
        double qy = x_(8);
        double qz = x_(9);
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        
        x_(6) = qw / q_norm;
        x_(7) = qx / q_norm;
        x_(8) = qy / q_norm;
        x_(9) = qz / q_norm;
        
        // 日誌輸出
        ROS_DEBUG("Odometry update: pos_innov=%.3f,%.3f,%.3f vel_innov=%.3f,%.3f,%.3f",
                 y_pos(0), y_pos(1), y_pos(2), y_vel(0), y_vel(1), y_vel(2));
    }
}; // 添加這個閉合大括號來結束 PX4EKFSimple 類定義

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_ekf_simple");
    
    PX4EKFSimple ekf;
    
    ros::spin();
    
    return 0;
} 