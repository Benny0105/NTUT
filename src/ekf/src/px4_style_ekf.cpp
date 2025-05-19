#include <ekf/px4_style_ekf.h>

namespace ekf {

PX4StyleEKF::PX4StyleEKF() : 
    nh_(),
    private_nh_("~"),
    initialized_(false),
    gps_origin_initialized_(false),
    first_update_(true),
    first_baro_received_(false)
{
    // 讀取基本參數
    private_nh_.param<double>("freq", freq_, 50.0);
    private_nh_.param<double>("sensor_timeout", sensor_timeout_, 0.5);
    private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
    private_nh_.param<std::string>("global_frame_id", global_frame_id_, "map");
    private_nh_.param<std::string>("output_frame_id", output_frame_id_, "odom");
    
    // 讀取傳感器控制參數
    private_nh_.param<bool>("use_imu", use_imu_, true);
    private_nh_.param<bool>("use_gps", use_gps_, true);
    private_nh_.param<bool>("use_mag", use_mag_, true);
    private_nh_.param<bool>("use_baro", use_baro_, true);
    private_nh_.param<bool>("use_odom", use_odom_, false);
    private_nh_.param<bool>("publish_tf", publish_tf_, true);
    
    // 讀取延遲參數
    private_nh_.param<double>("imu_delay", imu_delay_, 0.0);
    private_nh_.param<double>("gps_delay", gps_delay_, 0.0);
    private_nh_.param<double>("mag_delay", mag_delay_, 0.0);
    private_nh_.param<double>("baro_delay", baro_delay_, 0.0);
    private_nh_.param<double>("odom_delay", odom_delay_, 0.0);
    
    // 計算最大延遲
    max_delay_ = std::max({imu_delay_, gps_delay_, mag_delay_, baro_delay_, odom_delay_});
    
    // 讀取互補濾波器參數
    private_nh_.param<double>("tau_vel", tau_vel_, 10.0);
    private_nh_.param<double>("tau_pos", tau_pos_, 100.0);
    
    // 讀取噪聲參數
    private_nh_.param<double>("gyro_noise", gyro_noise_, 0.001);
    private_nh_.param<double>("accel_noise", accel_noise_, 0.01);
    private_nh_.param<double>("gps_pos_noise_x", gps_pos_noise_x_, 0.5);
    private_nh_.param<double>("gps_pos_noise_y", gps_pos_noise_y_, 0.5);
    private_nh_.param<double>("gps_pos_noise_z", gps_pos_noise_z_, 0.7);
    private_nh_.param<double>("gps_vel_noise", gps_vel_noise_, 0.1);
    private_nh_.param<double>("mag_noise", mag_noise_, 0.01);
    private_nh_.param<double>("baro_noise", baro_noise_, 0.5);
    
    // 讀取創新門限參數
    private_nh_.param<double>("pos_innov_gate", pos_innov_gate_, 5.0);
    private_nh_.param<double>("vel_innov_gate", vel_innov_gate_, 3.0);
    private_nh_.param<double>("hgt_innov_gate", hgt_innov_gate_, 3.0);
    private_nh_.param<double>("mag_innov_gate", mag_innov_gate_, 3.0);
    
    // 讀取PX4風格控制參數
    private_nh_.param<int>("imu_control", imu_control_, 3);  // 位掩碼: bit 0 - acc, bit 1 - gyro
    private_nh_.param<int>("gps_control", gps_control_, 7);  // 位掩碼: bit 0 - pos, bit 1 - vel, bit 2 - hgt
    private_nh_.param<int>("baro_control", baro_control_, 1); // 氣壓計融合標誌
    private_nh_.param<int>("mag_control", mag_control_, 7);  // 位掩碼: bit 0 - mag, bit 1 - heading, bit 2 - declination
    
    // 高度傳感器選擇
    private_nh_.param<int>("height_sensor", height_sensor_, 0); // 0=baro, 1=GPS, 2=range, 3=vision
    
    // 初始化發布者
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("velocity", 10);
    
    // 初始化訂閱者
    if (use_imu_) {
        imu_sub_ = nh_.subscribe("/imu/data", 10, &PX4StyleEKF::imuCallback, this);
    }
    
    if (use_gps_) {
        gps_sub_ = nh_.subscribe("/fix", 10, &PX4StyleEKF::gpsCallback, this);
    }
    
    if (use_mag_) {
        mag_sub_ = nh_.subscribe("/imu/mag", 10, &PX4StyleEKF::magCallback, this);
    }
    
    if (use_baro_) {
        baro_sub_ = nh_.subscribe("/baro", 10, &PX4StyleEKF::baroCallback, this);
    }
    
    if (use_odom_) {
        gps_vel_sub_ = nh_.subscribe("/fix_velocity", 10, &PX4StyleEKF::gpsVelCallback, this);
    }
    
    // 初始化更新定時器
    double timer_period = 1.0 / freq_;
    update_timer_ = nh_.createTimer(ros::Duration(timer_period), &PX4StyleEKF::updateTimerCallback, this);
    
    // 初始化TF廣播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    
    // 初始化其他成員變量
    gps_origin_initialized_ = false;
    initialized_ = false;
    first_update_ = true;
    first_baro_received_ = false;
    
    // 初始化GPS原點相關變量
    has_origin_ = false;
    has_gps_ = false;
    has_gps_vel_ = false;
    has_baro_ref_ = false;
    
    // 初始化最後更新時間
    last_update_time_ = ros::Time::now();
    
    // 讀取座標系統設置
    private_nh_.param<std::string>("coordinate_system", coordinate_system_, "ned");
    
    ROS_INFO("PX4 Style EKF initialized with freq=%.1f Hz", freq_);
}

PX4StyleEKF::~PX4StyleEKF()
{
    ROS_INFO("PX4 Style EKF shutting down");
}

void PX4StyleEKF::initialize()
{
    ROS_INFO("Initializing PX4 Style EKF state and covariance");
    
    // 初始化狀態向量 (16-dimension state vector)
    // [0-2]  : position (NED)
    // [3-5]  : velocity (NED)
    // [6-9]  : orientation quaternion (w,x,y,z)
    // [10-12]: gyroscope bias
    // [13-15]: accelerometer bias
    
    // 使用EkfState結構體初始化
    state_.position = Eigen::Vector3d::Zero();
    state_.velocity = Eigen::Vector3d::Zero();
    state_.quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  // w, x, y, z
    state_.gyro_bias = Eigen::Vector3d::Zero();
    state_.accel_bias = Eigen::Vector3d::Zero();
    
    // 初始化協方差矩陣
    P_ = Eigen::MatrixXd::Zero(16, 16);
    
    // 位置不確定性
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 10.0;
    
    // 速度不確定性
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0;
    
    // 姿態不確定性
    P_.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() * 0.1;
    
    // 陀螺儀偏置不確定性
    P_.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity() * 0.01;
    
    // 加速度計偏置不確定性
    P_.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity() * 0.1;
    
    // 初始化融合狀態
    fusion_status_ = FusionStatus();
    
    // 設置初始化標誌
    initialized_ = true;
    first_update_ = true;
    
    // 記錄當前時間
    last_update_time_ = ros::Time::now();
    
    // 讀取座標系統設置
    private_nh_.param<std::string>("coordinate_system", coordinate_system_, "ned");
    
    ROS_INFO("PX4 Style EKF initialized successfully with coordinate system: %s", coordinate_system_.c_str());
}

void PX4StyleEKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 檢查數據有效性
    if (std::isnan(msg->linear_acceleration.x) || std::isnan(msg->linear_acceleration.y) || std::isnan(msg->linear_acceleration.z) ||
        std::isnan(msg->angular_velocity.x) || std::isnan(msg->angular_velocity.y) || std::isnan(msg->angular_velocity.z)) {
        ROS_WARN("Invalid IMU data received with NaN values");
        return;
    }

    // 增加IMU數據有效性檢查
    double accel_norm = sqrt(
        msg->linear_acceleration.x * msg->linear_acceleration.x +
        msg->linear_acceleration.y * msg->linear_acceleration.y +
        msg->linear_acceleration.z * msg->linear_acceleration.z);
        
    double gyro_norm = sqrt(
        msg->angular_velocity.x * msg->angular_velocity.x +
        msg->angular_velocity.y * msg->angular_velocity.y +
        msg->angular_velocity.z * msg->angular_velocity.z);
        
    // 檢查IMU數據是否在合理範圍內
    const double MIN_ACCEL_NORM = 1.0;   // 最小加速度為1 m/s²
    const double MAX_ACCEL_NORM = 30.0;  // 最大加速度為30 m/s²（大約3G）
    const double MAX_GYRO_NORM = 10.0;   // 最大角速度為10 rad/s（~573°/s）
    
    bool imu_data_valid = (accel_norm >= MIN_ACCEL_NORM && 
                           accel_norm <= MAX_ACCEL_NORM && 
                           gyro_norm <= MAX_GYRO_NORM);
    
    if (!imu_data_valid) {
        static int invalid_imu_count = 0;
        invalid_imu_count++;
        
        // 如果超過20次連續不合理的IMU數據，發出警告
        if (invalid_imu_count > 20) {
            ROS_WARN_THROTTLE(1.0, "Suspicious IMU data: accel_norm=%.2f m/s², gyro_norm=%.2f rad/s", 
                            accel_norm, gyro_norm);
            invalid_imu_count = 0;
        }
        
        // 即使數據可疑，我們仍然處理它，但可能會進行額外的處理或過濾
        // 不要立即返回，以避免丟失過多的IMU數據
    } else {
        // 重置無效計數器
        static int invalid_imu_count = 0;
        invalid_imu_count = 0;
    }

    // 檢查IMU融合是否啟用
    if ((imu_control_ & 0x1) == 0) {
        ROS_DEBUG_THROTTLE(10.0, "IMU fusion is disabled");
        return;
    }

    // 初始化最後IMU時間，如果這是第一個IMU消息
    if (last_imu_time_.isZero()) {
        last_imu_time_ = msg->header.stamp;
        last_imu_msg_ = msg;
        ROS_INFO("First IMU message received");
        updateWithIMU(msg);
        return;
    }

    // 使用消息的時間戳來檢查超時，而不是當前時間
    double dt = (msg->header.stamp - last_imu_time_).toSec();
    
    // 檢查時間戳是否後退（可能由於多個發布者或時間同步問題）
    if (dt < -0.01) {
        ROS_WARN("IMU message timestamps not monotonic: %.6f sec backwards jump", -dt);
        // 對於時間戳後退，仍然接受數據但更新時間戳基準
        last_imu_time_ = msg->header.stamp;
        last_imu_msg_ = msg;
        updateWithIMU(msg);
        return;
    }
    
    // 檢查時間間隔是否過大
    if (dt > 30 * (1.0 / freq_)) {
        ROS_WARN_THROTTLE(5.0, "IMU data timeout - gap of %.3f sec between messages", dt);
    }

    // 更新最後IMU時間戳
    last_imu_time_ = msg->header.stamp;
    
    // 存儲最新的IMU消息
    last_imu_msg_ = msg;

    // 使用IMU數據更新
    updateWithIMU(msg);
}

void PX4StyleEKF::updateWithIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 檢查EKF是否已初始化
    if (!initialized_) {
        ROS_WARN_THROTTLE(5.0, "EKF not initialized yet, skipping IMU update");
        // 如果還沒初始化，這裡我們可以考慮進行初始化
        initialize();
        return;
    }
    
    // 檢查IMU融合是否啟用
    if ((imu_control_ & 0x1) == 0) {
        return;
    }
    
    // 獲取IMU數據
    Eigen::Vector3d accel(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    
    Eigen::Vector3d gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
    
    // 應用簡單的中值濾波器以減少噪聲 - 保存最後3個IMU讀數
    static std::deque<Eigen::Vector3d> accel_buffer;
    static std::deque<Eigen::Vector3d> gyro_buffer;
    
    accel_buffer.push_back(accel);
    gyro_buffer.push_back(gyro);
    
    if (accel_buffer.size() > 3) accel_buffer.pop_front();
    if (gyro_buffer.size() > 3) gyro_buffer.pop_front();
    
    // 如果有足夠的樣本，應用中值濾波
    if (accel_buffer.size() >= 3) {
        // 將三個讀數排序進行簡單的中值濾波
        std::vector<double> accel_x_values, accel_y_values, accel_z_values;
        std::vector<double> gyro_x_values, gyro_y_values, gyro_z_values;
        
        for (const auto& a : accel_buffer) {
            accel_x_values.push_back(a(0));
            accel_y_values.push_back(a(1));
            accel_z_values.push_back(a(2));
        }
        
        for (const auto& g : gyro_buffer) {
            gyro_x_values.push_back(g(0));
            gyro_y_values.push_back(g(1));
            gyro_z_values.push_back(g(2));
        }
        
        // 對每個分量單獨進行排序
        std::sort(accel_x_values.begin(), accel_x_values.end());
        std::sort(accel_y_values.begin(), accel_y_values.end());
        std::sort(accel_z_values.begin(), accel_z_values.end());
        std::sort(gyro_x_values.begin(), gyro_x_values.end());
        std::sort(gyro_y_values.begin(), gyro_y_values.end());
        std::sort(gyro_z_values.begin(), gyro_z_values.end());
        
        // 取中間值
        size_t mid_index = accel_x_values.size() / 2;
        accel(0) = accel_x_values[mid_index];
        accel(1) = accel_y_values[mid_index];
        accel(2) = accel_z_values[mid_index];
        gyro(0) = gyro_x_values[mid_index];
        gyro(1) = gyro_y_values[mid_index];
        gyro(2) = gyro_z_values[mid_index];
    }
    
    // 檢查車輛是否靜止 - 通過計算加速度和角速度的范數
    double accel_norm = accel.norm();
    double gyro_norm = gyro.norm();
    
    // 記錄處理後的IMU數據狀態，便於診斷
    ROS_DEBUG_THROTTLE(5.0, "Processed IMU data: accel=[%.2f, %.2f, %.2f], gyro=[%.2f, %.2f, %.2f], accel_norm=%.2f, gyro_norm=%.2f",
                       accel(0), accel(1), accel(2), gyro(0), gyro(1), gyro(2), accel_norm, gyro_norm);
                       
    // 每隔一段時間記錄一次狀態，以便診斷
    static int imu_counter = 0;
    if (++imu_counter % 100 == 0) {  // 大約每秒記錄一次（假設IMU更新率為100Hz）
        ROS_INFO("IMU Health Check: accel_norm=%.2f m/s², gyro_norm=%.2f rad/s, status=%s",
                accel_norm, gyro_norm, (accel_norm >= 1.0 && accel_norm <= 30.0 && gyro_norm <= 10.0) ? "健康" : "異常");
    }
    
    // 改進的靜止狀態判斷標準 - 對於飛行器更加寬鬆
    bool acc_stationary = (accel_norm > 9.5 && accel_norm < 10.1);
    bool gyro_stationary = (gyro_norm < 0.05);  // 稍微提高閾值
    bool is_stationary = acc_stationary && gyro_stationary;
    
    // 如果車輛靜止，更新陀螺儀偏置並補償重力
    if (is_stationary) {
        // 獲取當前的陀螺儀偏置估計
        Eigen::Vector3d gyro_bias = state_.gyro_bias;
        
        // 計算新的偏置估計（使用自適應滑動平均）
        // 根據當前速度確定偏置估計的更新速率
        double bias_alpha = 0.002;  // 初始非常緩慢的更新率，但比之前稍微快
        
        // 檢查當前速度，決定是否更積極地更新偏置
        double velocity_norm = state_.velocity.norm();
        if (velocity_norm < 0.05) {
            // 幾乎靜止狀態，可以更積極地估計偏置
            bias_alpha = 0.02;  // 增加更新率
        }
        
        // 更新陀螺儀偏置
        gyro_bias = (1.0 - bias_alpha) * gyro_bias + bias_alpha * gyro;
        
        // 更新狀態向量中的陀螺儀偏置
        state_.gyro_bias = gyro_bias;
        
        // 改進的加速度計偏置估計
        // 當靜止時，加速度計應該只測量重力加速度
        Eigen::Vector3d gravity_vector(0.0, 0.0, 9.81);  // NED坐標系中重力沿Z軸正方向
        
        // 獲取當前的姿態四元數
        Eigen::Quaterniond q(state_.quaternion.w(), state_.quaternion.x(), state_.quaternion.y(), state_.quaternion.z());
        
        // 將重力向量轉換到機體坐標系
        Eigen::Vector3d expected_accel = q.inverse() * gravity_vector;
        
        // 計算測量的加速度和預期加速度之間的差異
        Eigen::Vector3d accel_bias_innovation = accel - expected_accel;
        
        // 獲取當前的加速度計偏置估計
        Eigen::Vector3d accel_bias = state_.accel_bias;
        
        // 非常慢速地更新加速度計偏置
        double accel_bias_alpha = 0.001;  // 稍微提高更新率
        accel_bias = (1.0 - accel_bias_alpha) * accel_bias + accel_bias_alpha * accel_bias_innovation;
        
        // 限制加速度計偏置的大小，避免過度補償
        double max_accel_bias = 0.8;  // 增大最大允許的加速度計偏置(m/s²)
        for (int i = 0; i < 3; i++) {
            if (std::abs(accel_bias(i)) > max_accel_bias) {
                accel_bias(i) = (accel_bias(i) > 0) ? max_accel_bias : -max_accel_bias;
            }
        }
        
        // 更新狀態向量中的加速度計偏置
        state_.accel_bias = accel_bias;
        
        // 在靜止狀態下，重置所有速度分量為零，避免漂移
        if (velocity_norm > 0.1) {  // 如果速度不是零
            ROS_INFO("Vehicle stationary, resetting velocity from [%.2f, %.2f, %.2f] to [0, 0, 0]",
                   state_.velocity(0), state_.velocity(1), state_.velocity(2));
            state_.velocity.setZero();  // 重置所有速度分量
        }
        
        ROS_DEBUG_THROTTLE(5.0, "Vehicle stationary, updated biases: gyro=[%.6f, %.6f, %.6f], accel=[%.6f, %.6f, %.6f]",
                          gyro_bias(0), gyro_bias(1), gyro_bias(2),
                          accel_bias(0), accel_bias(1), accel_bias(2));
    }
    
    // 在仿真環境可能會使用IMU數據進行狀態更新（在真實系統中，這已經在predict中處理）
    
    ROS_DEBUG_THROTTLE(5.0, "IMU update processed: accel=[%.2f, %.2f, %.2f], gyro=[%.2f, %.2f, %.2f]",
                      accel(0), accel(1), accel(2), gyro(0), gyro(1), gyro(2));
}

void PX4StyleEKF::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // 檢查數據有效性
    if (msg == nullptr) {
        ROS_WARN("Received null GPS message.");
        return;
    }
    
    // 檢查傳感器超時
    ros::Time current_time = msg->header.stamp;
    if ((current_time - last_gps_time_).toSec() > sensor_timeout_) {
        ROS_WARN("GPS data timeout.");
    }
    last_gps_time_ = current_time;
    
    if (!initialized_) {
        ROS_WARN("EKF not initialized yet, skipping GPS update.");
        return;
    }
    
    // 處理GPS數據
    updateWithGPS(msg);
}

void PX4StyleEKF::magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    // 檢查磁場數據合理性
    Eigen::Vector3d mag_field(
        msg->magnetic_field.x,
        msg->magnetic_field.y,
        msg->magnetic_field.z
    );
    
    // 計算磁場強度
    double mag_strength = mag_field.norm();
    
    // 合理的磁場強度範圍（單位：特斯拉）
    const double MIN_MAG_STRENGTH = 0.1e-6;  // 0.1 μT
    const double MAX_MAG_STRENGTH = 100e-6;  // 100 μT
    
    if (mag_strength < MIN_MAG_STRENGTH || mag_strength > MAX_MAG_STRENGTH) {
        ROS_WARN_THROTTLE(5.0, "Unusual magnetic field strength: %.2f μT, ignoring measurement", 
                         mag_strength * 1e6);
        return;
    }
    
    // 從四元數獲取當前姿態
    Eigen::Quaterniond q(state_.quaternion.w(), state_.quaternion.x(), state_.quaternion.y(), state_.quaternion.z());
    Eigen::Matrix3d R_body_to_ned = q.toRotationMatrix();
    
    // 獲取當前roll和pitch角
    double roll = std::atan2(R_body_to_ned(2, 1), R_body_to_ned(2, 2));
    double pitch = std::asin(-R_body_to_ned(2, 0));
    
    // 從機體坐標系轉換到水平平面
    // 首先計算只有roll和pitch的旋轉矩陣（不包含yaw）
    Eigen::Matrix3d R_roll_pitch = 
        (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
    
    // 將磁場向量旋轉到水平坐標系
    Eigen::Vector3d mag_horizontal = R_roll_pitch * mag_field;
    
    // 計算磁場在水平平面的投影
    double mag_x = mag_horizontal(0);
    double mag_y = mag_horizontal(1);
    
    // 計算磁北方向的航向角
    double measured_yaw = std::atan2(mag_y, mag_x);
    
    // 應用磁偏角修正（通常需要根據當地磁偏角進行修正）
    double magnetic_declination = 0.0;  // 應該從配置中讀取或使用地理位置計算
    double true_yaw = measured_yaw - magnetic_declination;
    
    // 將角度規範化到[-pi, pi]範圍內
    while (true_yaw > M_PI) true_yaw -= 2.0 * M_PI;
    while (true_yaw < -M_PI) true_yaw += 2.0 * M_PI;
    
    // 獲取當前的yaw角
    double current_yaw = std::atan2(R_body_to_ned(1, 0), R_body_to_ned(0, 0));
    
    // 計算yaw角的差異（創新）
    double yaw_innovation = true_yaw - current_yaw;
    
    // 將yaw創新規範化到[-pi, pi]範圍
    while (yaw_innovation > M_PI) yaw_innovation -= 2.0 * M_PI;
    while (yaw_innovation < -M_PI) yaw_innovation += 2.0 * M_PI;
    
    // 動態調整磁力計的權重 - 根據傾斜角調整Kalman增益
    // 當roll和pitch角較大時，減小磁力計的權重
    double tilt_angle = std::sqrt(roll*roll + pitch*pitch);
    const double MAX_TILT_FOR_MAG = 60.0 * M_PI / 180.0;  // 60度最大傾角
    
    double mag_gain = 0.05;  // 默認增益
    
    if (tilt_angle > MAX_TILT_FOR_MAG) {
        // 高傾角時減少磁力計影響
        mag_gain *= 0.1;
        ROS_DEBUG_THROTTLE(1.0, "Large tilt angle (%.1f deg), reducing mag influence", 
                          tilt_angle * 180.0 / M_PI);
    } else {
        // 根據傾斜角線性調整增益
        mag_gain *= (1.0 - tilt_angle / MAX_TILT_FOR_MAG * 0.8);
    }
    
    // 如果創新太大，可能是磁力計異常或快速轉彎
    if (std::abs(yaw_innovation) > 0.5) {  // 超過~30度
        // 進一步減小增益
        mag_gain *= 0.5;
        ROS_DEBUG_THROTTLE(1.0, "Large yaw innovation (%.1f deg), reducing impact",
                          yaw_innovation * 180.0 / M_PI);
    }
    
    // 使用磁力計測量更新航向角
    double new_yaw = current_yaw + yaw_innovation * mag_gain;
    
    // 構建更新後的四元數
    Eigen::Quaterniond updated_q = 
        Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    // 更新狀態中的四元數
    state_.quaternion.w() = updated_q.w();
    state_.quaternion.x() = updated_q.x();
    state_.quaternion.y() = updated_q.y();
    state_.quaternion.z() = updated_q.z();
    
    // 記錄磁力計使用狀態
    fusion_status_.mag_fusion_active = true;
    
    // 添加調試信息
    ROS_DEBUG_THROTTLE(2.0, "Mag update: measured_yaw=%.1f deg, current_yaw=%.1f deg, "
                      "innovation=%.1f deg, gain=%.3f, tilt=%.1f deg",
                      measured_yaw * 180.0 / M_PI, current_yaw * 180.0 / M_PI,
                      yaw_innovation * 180.0 / M_PI, mag_gain, tilt_angle * 180.0 / M_PI);
                      
    // 每秒打印一次詳細的磁力計處理信息
    static int mag_counter = 0;
    if (++mag_counter % 10 == 0) {
        ROS_INFO("Mag update: field=[%.2f, %.2f, %.2f] μT, strength=%.2f μT, "
                "heading=%.1f deg, innovation=%.1f deg", 
                mag_field(0)*1e6, mag_field(1)*1e6, mag_field(2)*1e6, mag_strength*1e6,
                true_yaw * 180.0 / M_PI, yaw_innovation * 180.0 / M_PI);
    }
}

void PX4StyleEKF::baroCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // 檢查數據有效性
    if (msg == nullptr) {
        ROS_WARN("Received null barometer message.");
        return;
    }
    
    // 檢查傳感器超時
    ros::Time current_time = ros::Time::now(); // 假設時間戳是當前時間
    if ((current_time - last_baro_time_).toSec() > sensor_timeout_) {
        ROS_WARN("Barometer data timeout.");
    }
    last_baro_time_ = current_time;
    
    if (!initialized_) {
        ROS_WARN("EKF not initialized yet, skipping barometer update.");
        return;
    }
    
    // 處理氣壓計數據
    updateWithBaro(msg);
}

double PX4StyleEKF::getEarthRadius()
{
    // 返回地球的平均半徑，單位為米
    return 6371000.0;
}

void PX4StyleEKF::convertLLAtoNED(double lat, double lon, double alt, double& north, double& east, double& down)
{
    // 轉換WGS-84緯度/經度/高度到北東地坐標系（NED）
    // 使用平面地球近似，對於小範圍內的位置很有效
    
    // 獲取地球半徑
    double earth_radius = getEarthRadius();
    
    // 將緯度和經度從度轉換為弧度
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double lat_origin_rad = gps_origin_lat_ * M_PI / 180.0;
    double lon_origin_rad = gps_origin_lon_ * M_PI / 180.0;
    
    // 計算弧度差異
    double d_lat = lat_rad - lat_origin_rad;
    double d_lon = lon_rad - lon_origin_rad;
    
    // 計算北向和東向距離
    north = earth_radius * d_lat;
    east = earth_radius * cos(lat_origin_rad) * d_lon;
    
    // 計算下向距離（高度差異，考慮NED坐標系中down方向為正）
    down = gps_origin_alt_ - alt;
    
    ROS_DEBUG_THROTTLE(5.0, "LLA to NED conversion: lat=%.7f, lon=%.7f, alt=%.2f -> N=%.2f, E=%.2f, D=%.2f",
                      lat, lon, alt, north, east, down);
}

void PX4StyleEKF::updateTimerCallback(const ros::TimerEvent& event)
{
    // 檢查EKF是否已初始化
    if (!initialized_) {
        ROS_WARN_THROTTLE(5.0, "EKF not initialized yet, skipping timer update");
        return;
    }
    
    // 檢查是否已接收IMU數據
    if (!last_imu_msg_) {
        ROS_WARN_THROTTLE(5.0, "No IMU data received yet, skipping timer update");
        return;
    }
    
    // 獲取最新IMU消息的時間戳，用作當前時間
    ros::Time current_time = last_imu_msg_->header.stamp;
    
    // 執行預測步驟
    predict(current_time);
    
    // 檢查並重置姿態（如果需要）
    resetAttitudeIfNeeded();
    
    // 發布當前狀態
    publishState();
    
    ROS_DEBUG("Timer update completed with IMU timestamp: %.3f s", current_time.toSec());
}

void PX4StyleEKF::predict(const ros::Time& current_time)
{
    // 檢查EKF是否已初始化
    if (!initialized_) {
        ROS_WARN("EKF not initialized, skipping prediction step");
        return;
    }
    
    // 檢查是否已接收IMU數據
    if (!last_imu_msg_) {
        ROS_WARN_THROTTLE(1.0, "No IMU data received yet, skipping prediction step");
        return;
    }
    
    // 獲取IMU消息的時間戳，應該與last_imu_time_一致
    ros::Time imu_time = last_imu_msg_->header.stamp;
    
    // 如果這是第一次預測，就以IMU時間為基準
    if (first_update_) {
        last_update_time_ = imu_time;
        first_update_ = false;
        ROS_INFO("First prediction using IMU timestamp: %.6f", imu_time.toSec());
        return;
    }
    
    // 計算時間差（使用IMU的時間戳而不是系統時間）
    double dt = (imu_time - last_update_time_).toSec();
    
    // 檢查時間差是否有效
    if (dt <= 0.0) {
        // 時間戳後退，可能是由於接收到的IMU數據時間順序問題
        ROS_WARN("Time went backwards in predict step: dt=%.6f s, resetting time reference", dt);
        last_update_time_ = imu_time;
        return;
    } else if (dt > 5.0) {
        // 時間差過大，可能是由於長時間沒收到數據或時間跳躍
        ROS_WARN("Large time delta in predict: %.6f s, limiting to 0.1s", dt);
        dt = 0.1; // 限制最大時間步長，避免大的狀態跳變
    }
    
    // 如果時間間隔太短，忽略此次更新
    if (dt < 0.001) {
        return;
    }
    
    // 獲取當前狀態
    Eigen::Vector3d position = state_.position;
    Eigen::Vector3d velocity = state_.velocity;
    Eigen::Quaterniond q(state_.quaternion.w(), state_.quaternion.x(), state_.quaternion.y(), state_.quaternion.z());
    q.normalize();
    
    // 獲取當前偏置
    Eigen::Vector3d gyro_bias = state_.gyro_bias;
    Eigen::Vector3d accel_bias = state_.accel_bias;
    
    // 獲取最新的IMU數據
    sensor_msgs::Imu last_imu_msg = *last_imu_msg_;
    
    // 獲取加速度和角速度並補償偏置
    Eigen::Vector3d accel(
        last_imu_msg.linear_acceleration.x - accel_bias(0),
        last_imu_msg.linear_acceleration.y - accel_bias(1),
        last_imu_msg.linear_acceleration.z - accel_bias(2));
    
    Eigen::Vector3d gyro(
        last_imu_msg.angular_velocity.x - gyro_bias(0),
        last_imu_msg.angular_velocity.y - gyro_bias(1),
        last_imu_msg.angular_velocity.z - gyro_bias(2));
    
    // 將加速度從機體坐標系轉換到NED坐標系
    Eigen::Matrix3d R_body_to_ned = q.toRotationMatrix();
    Eigen::Vector3d accel_ned = R_body_to_ned * accel;
    
    // 改進的靜止檢測邏輯 - 更適合飛行器
    double accel_norm = accel.norm();
    double gyro_norm = gyro.norm();
    
    // 更寬鬆的靜止狀態檢測條件，避免過度判定靜止
    bool acc_stationary = (accel_norm > 9.0 && accel_norm < 10.5 && 
                          std::abs(accel(0)) < 0.2 && std::abs(accel(1)) < 0.2);  // 更關注水平加速度分量
    bool gyro_stationary = (gyro_norm < 0.15);  // 允許更大的角速度變化
    bool is_stationary = acc_stationary && gyro_stationary;
    
    // 在飛行過程中禁用靜止檢測，如果高度已經改變了一定值（預設為超過2米就不再判定為靜止）
    static double initial_height = 0.0;
    static bool height_initialized = false;
    
    if (!height_initialized && initialized_) {
        initial_height = state_.position(2);
        height_initialized = true;
        ROS_INFO("Initial height set to %.2f m for stationary detection", initial_height);
    }
    
    // 如果高度變化超過2米，禁用靜止檢測
    if (height_initialized && std::abs(state_.position(2) - initial_height) > 2.0) {
        // 在飛行中，不應該判定為靜止
        is_stationary = false;
    }
    
    // 重力補償處理 - 永遠補償重力，即使在靜止狀態
    // 在NED系統中，重力向下為正
    accel_ned(2) += 9.81;  // 總是添加重力補償

    // 在靜止狀態下，處理特殊情況
    if (is_stationary) {
        ROS_DEBUG_THROTTLE(5.0, "Vehicle detected as stationary");
        // 在靜止狀態下，慢慢減小加速度
        accel_ned *= 0.1;  // 而不是完全設為零
    }
    
    // 預測位置: p = p + v*dt + 0.5*a*dt^2
    position += velocity * dt + 0.5 * accel_ned * dt * dt;
    
    // 預測速度: v = v + a*dt
    velocity += accel_ned * dt;
    
    // 改進的速度處理邏輯 - 提高靈敏度
    if (is_stationary) {
        // 在靜止狀態下慢慢減小速度而不是立即重置為零
        velocity *= 0.8;  // 指數衰減
        
        // 如果速度已經很小，則直接設為零
        if (velocity.norm() < 0.01) {  // 降低閾值從0.02到0.01
            velocity.setZero();
        }
    } else {
        // 非靜止狀態使用更小的過濾閾值，提高速度估計的靈敏度
        double vel_threshold = 0.001;  // 顯著降低閾值以提高速度感知
        for (int i = 0; i < 3; i++) {
            if (std::abs(velocity(i)) < vel_threshold) {
                velocity(i) = 0.0;
            }
        }
        
        // 為了避免速度估計不足，如果IMU加速度明顯但速度估計很小，則增加速度
        double accel_horizontal_norm = sqrt(accel_ned(0)*accel_ned(0) + accel_ned(1)*accel_ned(1));
        double vel_horizontal_norm = sqrt(velocity(0)*velocity(0) + velocity(1)*velocity(1));
        
        if (accel_horizontal_norm > 0.5 && vel_horizontal_norm < 0.1) {
            // 有明顯的水平加速度但速度估計很小，可能低估了速度
            double accel_boost = std::min(1.0, accel_horizontal_norm * 0.2); // 最多增加1倍
            velocity(0) *= (1.0 + accel_boost);
            velocity(1) *= (1.0 + accel_boost);
            ROS_INFO_THROTTLE(1.0, "Boosting velocity estimate due to acceleration: accel=%.2f m/s², boost=%.2f",
                           accel_horizontal_norm, accel_boost);
        }
    }
    
    // 預測姿態 (使用四元數積分)
    if (gyro_norm > 1e-12) {
        // 使用四元數積分的改進方法（Runge-Kutta法）以提高精度
        Eigen::Vector3d gyro_unbiased = gyro - gyro_bias;
        
        // 使用更精確的四元數積分方式
        // 建立角速度四元數
        Eigen::Quaterniond omega_q(0.0, gyro_unbiased(0), gyro_unbiased(1), gyro_unbiased(2));
        
        // 使用簡化的積分方法
        Eigen::Quaterniond qdot = omega_q * q;
        qdot.coeffs() *= 0.5;
        
        // 使用一階歐拉積分
        q.coeffs() += qdot.coeffs() * dt;
        
        // 確保四元數規範化，防止數值誤差累積
        q.normalize();
    }
    
    // 更新狀態向量
    state_.position = position;
    state_.velocity = velocity;
    state_.quaternion = q;
    
    // 更新協方差矩陣 - 動態調整過程噪聲
    double pos_process_noise = is_stationary ? 0.001 : 0.01;  // 靜止時降低位置噪聲
    double vel_process_noise = is_stationary ? 0.0001 : 0.1;  // 靜止時大幅降低速度噪聲
    
    // 位置和速度不確定性隨時間增長
    for (int i = 0; i < 3; ++i) {
        P_(i, i) += pos_process_noise * dt * dt;        // 位置協方差增長
        P_(i+3, i+3) += vel_process_noise * dt;         // 速度協方差增長
    }
    
    // 偏置不確定性緩慢增長 (隨機遊走模型)
    for (int i = 0; i < 3; ++i) {
        P_(i+10, i+10) += 1e-12 * dt;  // 陀螺儀偏置協方差增長
        P_(i+13, i+13) += 1e-10 * dt;  // 加速度計偏置協方差增長
    }
    
    // 更新最後更新時間（使用IMU時間戳）
    last_update_time_ = imu_time;
    
    ROS_DEBUG("Predicted state: pos=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f]",
              position(0), position(1), position(2),
              velocity(0), velocity(1), velocity(2));
}

void PX4StyleEKF::publishState()
{
    // Current time
    ros::Time current_time = ros::Time::now();
    
    // Publish pose (with covariance)
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = global_frame_id_;
    
    // Set position
    pose_msg.pose.pose.position.x = state_.position(0);
    pose_msg.pose.pose.position.y = state_.position(1);
    pose_msg.pose.pose.position.z = state_.position(2);
    
    // Set orientation
    pose_msg.pose.pose.orientation.x = state_.quaternion.x();
    pose_msg.pose.pose.orientation.y = state_.quaternion.y();
    pose_msg.pose.pose.orientation.z = state_.quaternion.z();
    pose_msg.pose.pose.orientation.w = state_.quaternion.w();
    
    // Set covariance (position and orientation)
    // The covariance is in the order: [x, y, z, rotation about X, rotation about Y, rotation about Z]
    // We need to copy from our separate covariances
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Position covariance
            pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
            
            // Orientation covariance
            pose_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = P_(i + 3, j + 3);
        }
    }
    
    // Publish
    pose_pub_.publish(pose_msg);
    
    // 發布PoseStamped消息以兼容需要此類型的客戶端
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = pose_msg.header;
    pose_stamped_msg.pose = pose_msg.pose.pose;
    
    // 首先發布PoseStamped，確保客戶端能夠接收到正確的位姿信息
    pose_stamped_pub_.publish(pose_stamped_msg);
    
    // 添加日誌，用於調試客戶端連接問題
    ROS_DEBUG_THROTTLE(10.0, "Published PoseStamped on topic %s", pose_stamped_pub_.getTopic().c_str());
    
    // Publish velocity
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = current_time;
    vel_msg.header.frame_id = base_frame_id_;
    
    // Set linear velocity
    vel_msg.twist.linear.x = state_.velocity(0);
    vel_msg.twist.linear.y = state_.velocity(1);
    vel_msg.twist.linear.z = state_.velocity(2);
    
    // Set angular velocity from IMU (if available)
    if (last_imu_msg_) {
        if (coordinate_system_ == "ned") {
            // In NED, we need to convert from ENU
            vel_msg.twist.angular.x = last_imu_msg_->angular_velocity.y;
            vel_msg.twist.angular.y = last_imu_msg_->angular_velocity.x; 
            vel_msg.twist.angular.z = -last_imu_msg_->angular_velocity.z;
        } else {
            // In ENU, just copy
            vel_msg.twist.angular = last_imu_msg_->angular_velocity;
        }
    }
    
    // Publish velocity
    velocity_pub_.publish(vel_msg);
    
    // Publish odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = global_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    
    // Set position and orientation
    odom_msg.pose.pose = pose_msg.pose.pose;
    odom_msg.pose.covariance = pose_msg.pose.covariance;
    
    // Set velocity
    odom_msg.twist.twist = vel_msg.twist;
    
    // Set velocity covariance
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Velocity covariance
            odom_msg.twist.covariance[i * 6 + j] = P_(i + 3, j + 3);
        }
    }
    
    // Publish odometry
    odom_pub_.publish(odom_msg);
    
    // Publish transform if enabled
    if (publish_tf_) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = global_frame_id_;
        transform.child_frame_id = base_frame_id_;
        
        // Set translation
        transform.transform.translation.x = state_.position(0);
        transform.transform.translation.y = state_.position(1);
        transform.transform.translation.z = state_.position(2);
        
        // Set rotation
        transform.transform.rotation = pose_msg.pose.pose.orientation;
        
        // Publish transform
        tf_broadcaster_->sendTransform(transform);
    }
}

void PX4StyleEKF::updateWithGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // 檢查EKF是否已初始化
    if (!initialized_) {
        ROS_WARN_THROTTLE(5.0, "EKF not initialized yet, skipping GPS update");
        return;
    }
    
    // 檢查GPS融合是否啟用
    if ((gps_control_ & 0x1) == 0) {
        return;
    }
    
    // 檢查GPS修正狀態
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_THROTTLE(5.0, "No GPS fix, skipping update");
        return;
    }
    
    // 檢查GPS數據有效性
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude) ||
        std::isinf(msg->latitude) || std::isinf(msg->longitude) || std::isinf(msg->altitude)) {
        ROS_WARN_THROTTLE(5.0, "Invalid GPS data: lat=%.7f, lon=%.7f, alt=%.2f",
                       msg->latitude, msg->longitude, msg->altitude);
        return;
    }
    
    // 將GPS位置從全球坐標系轉換到NED坐標系
    geographic_to_NED(msg->latitude, msg->longitude, msg->altitude, local_e, local_n, local_d);
    
    // 更新GPS數據有效性標誌
    has_gps_ = true;
    
    // 在日誌中顯示位置信息，用於調試
    ROS_DEBUG_THROTTLE(5.0, "GPS Update: lat=%.7f, lon=%.7f, alt=%.2f -> NED: %.2f, %.2f, %.2f",
                      msg->latitude, msg->longitude, msg->altitude,
                      local_n, local_e, local_d);

    // 在初始化期間或每隔一段時間更新參考原點
    bool update_origin = false;
    
    if (!has_origin_) {
        update_origin = true;
        ROS_INFO("Setting initial GPS origin to lat=%.7f, lon=%.7f, alt=%.2f",
                msg->latitude, msg->longitude, msg->altitude);
    } else {
        // 每小時更新一次原點，避免長時間累積誤差
        static ros::Time last_origin_update = ros::Time::now();
        if ((ros::Time::now() - last_origin_update).toSec() > 3600.0) {
            update_origin = true;
            last_origin_update = ros::Time::now();
            ROS_INFO("Periodic update of GPS origin to lat=%.7f, lon=%.7f, alt=%.2f",
                    msg->latitude, msg->longitude, msg->altitude);
        }
    }
    
    if (update_origin) {
        latitude_origin_ = msg->latitude;
        longitude_origin_ = msg->longitude;
        altitude_origin_ = msg->altitude;
        has_origin_ = true;
    }
    
    // 使用GPS觀測來更新位置 - 使用更高的增益提高響應速度
    double position_gain = 0.3;  // 增加位置增益以更快地跟踪GPS位置
    state_.position(0) += position_gain * (local_n - state_.position(0));
    state_.position(1) += position_gain * (local_e - state_.position(1));
    state_.position(2) += position_gain * (local_d - state_.position(2));

    // 檢查高度數據有效性
    if (msg->altitude > -1000.0 && msg->altitude < 10000.0) {  // 合理的高度範圍檢查
        baro_ref_alt_ = msg->altitude;  // 更新氣壓計參考高度
        has_baro_ref_ = true;
    }
}

void PX4StyleEKF::updateWithBaro(const std_msgs::Float32::ConstPtr& msg)
{
    // 檢查EKF是否已初始化
    if (!initialized_) {
        ROS_WARN_THROTTLE(5.0, "EKF not initialized yet, skipping barometer update");
        return;
    }
    
    // 檢查氣壓計融合是否啟用
    if (baro_control_ == 0) {
        return;
    }
    
    // 檢查高度傳感器選擇
    if (height_sensor_ != 0) {
        // 如果氣壓計不是主要高度傳感器，可能會用於輔助或完全禁用
        if (height_sensor_ == 1) { // GPS是主要高度傳感器
            ROS_DEBUG_THROTTLE(10.0, "Barometer not used as primary height sensor");
            return;
        }
    }
    
    // 獲取氣壓計高度
    double baro_height = msg->data;
    
    // 檢查數據有效性
    if (std::isnan(baro_height) || std::isinf(baro_height)) {
        ROS_WARN_THROTTLE(5.0, "Invalid barometer height: %.2f m", baro_height);
        return;
    }
    
    // 第一次接收氣壓計數據，進行初始化
    if (!first_baro_received_) {
        ROS_INFO("Initializing barometer height reference: %.2f m", baro_height);
        baro_origin_height_ = baro_height;
        first_baro_received_ = true;
        
        // 如果氣壓計是主要高度傳感器，則初始化Z軸位置
        if (height_sensor_ == 0) {
            // 在NED坐標系中，下方向為正
            state_.position(2) = -(baro_height - baro_origin_height_);
            ROS_INFO("Initializing EKF height from barometer: %.2f m", baro_height);
        }
        return;
    }
    
    // 計算氣壓計測量的相對高度
    double relative_height = baro_height - baro_origin_height_;
    
    // 獲取當前狀態的垂直位置（NED系統中，z軸向下為正，所以對height取負）
    double current_height = -state_.position(2);
    
    // 計算高度創新（測量值 - 預測值）
    double height_innovation = relative_height - current_height;
    
    // 動態Kalman增益 - 基於創新大小調整
    double height_gain = 0.2; // 默認增益
    
    if (std::abs(height_innovation) > 10.0) {
        height_gain = 0.1; // 大創新時減小增益
        ROS_WARN_THROTTLE(5.0, "Large barometer innovation: %.2f m, reducing gain", height_innovation);
    }
    
    // 應用高度更新 (更新z軸狀態，NED系統中z軸向下為正)
    state_.position(2) += -height_innovation * height_gain;
    
    // 更新融合狀態
    fusion_status_.baro_fusion_active = true;
    
    ROS_DEBUG("Barometer update applied: height=%.2f m, innovation=%.2f m, correction=%.2f m", 
              relative_height, height_innovation, -height_innovation * height_gain);
}

// 添加一個新的輔助函數來規範化航向角
double PX4StyleEKF::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 添加姿態重置功能，解決姿態發散問題
void PX4StyleEKF::resetAttitudeIfNeeded()
{
    // 檢查四元數規範化
    Eigen::Quaterniond q(state_.quaternion.w(), state_.quaternion.x(), state_.quaternion.y(), state_.quaternion.z());
    double q_norm = q.norm();
    
    // 如果四元數范數偏離1太多，則進行規範化
    if (std::abs(q_norm - 1.0) > 0.005) {  // 降低閾值以更頻繁地進行規範化
        ROS_WARN("Quaternion norm deviation detected: %.4f, normalizing", q_norm);
        q.normalize();
        state_.quaternion.w() = q.w();
        state_.quaternion.x() = q.x();
        state_.quaternion.y() = q.y();
        state_.quaternion.z() = q.z();
    }
    
    // 提取歐拉角
    Eigen::Matrix3d rotation = q.toRotationMatrix();
    double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    double pitch = std::asin(-rotation(2, 0));
    double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    
    // 每隔一段時間打印當前姿態，以便診斷
    static int attitude_counter = 0;
    if (++attitude_counter % 50 == 0) {  // 約每0.5秒記錄一次（假設100Hz更新率）
        ROS_INFO("Current attitude: roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
               roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }
    
    // 檢查pitch角是否接近奇點(±90度)
    if (std::abs(std::abs(pitch) - M_PI/2) < 0.1) {
        ROS_WARN("Attitude near gimbal lock detected (pitch=%.1f deg), applying soft constraints", 
                pitch * 180.0 / M_PI);
                
        // 輕微調整pitch角遠離奇點
        double adjusted_pitch = pitch > 0 ? M_PI/2 - 0.1 : -M_PI/2 + 0.1;
        
        // 構建新的四元數
        Eigen::Quaterniond new_q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                                 Eigen::AngleAxisd(adjusted_pitch, Eigen::Vector3d::UnitY()) * 
                                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
                                 
        // 更新狀態
        state_.quaternion.w() = new_q.w();
        state_.quaternion.x() = new_q.x();
        state_.quaternion.y() = new_q.y();
        state_.quaternion.z() = new_q.z();
    }
    
    // 檢查姿態是否異常 - 通過檢測roll和pitch角是否過大
    // 無人機允許更大的傾斜角度
    double max_tilt_angle = 80.0 * M_PI / 180.0;  // 進一步增加最大允許的傾斜角度到80度
    
    if (std::abs(roll) > max_tilt_angle || std::abs(pitch) > max_tilt_angle) {
        // 姿態異常，計數器增加
        static int abnormal_attitude_count = 0;
        abnormal_attitude_count++;
        
        ROS_WARN_THROTTLE(1.0, "Abnormal attitude detected: roll=%.1f°, pitch=%.1f° (count=%d)", 
                        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, abnormal_attitude_count);
        
        // 如果連續多次檢測到異常姿態，進行部分重置
        if (abnormal_attitude_count > 30) {  // 進一步增加容忍度
            ROS_WARN("Multiple abnormal attitudes detected, partially resetting attitude");
            
            // 使用當前航向角，但限制roll和pitch在安全範圍內
            double limited_roll = std::max(std::min(roll, max_tilt_angle), -max_tilt_angle);
            double limited_pitch = std::max(std::min(pitch, max_tilt_angle), -max_tilt_angle);
            
            // 構建新的四元數
            Eigen::Quaterniond corrected_q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                                           Eigen::AngleAxisd(limited_pitch, Eigen::Vector3d::UnitY()) * 
                                           Eigen::AngleAxisd(limited_roll, Eigen::Vector3d::UnitX());
            
            // 更新狀態
            state_.quaternion.w() = corrected_q.w();
            state_.quaternion.x() = corrected_q.x();
            state_.quaternion.y() = corrected_q.y();
            state_.quaternion.z() = corrected_q.z();
            
            // 部分重置後，重置計數器
            abnormal_attitude_count = 0;
        }
    } else {
        // 姿態正常，重置計數器
        static int abnormal_attitude_count = 0;
        abnormal_attitude_count = 0;
    }
    
    // 如果沒有IMU數據或有其他問題，可以考慮重置姿態到安全值
    if (last_imu_msg_ == nullptr || 
        (std::isnan(state_.quaternion.w()) || std::isnan(state_.quaternion.x()) || 
         std::isnan(state_.quaternion.y()) || std::isnan(state_.quaternion.z()))) {
        ROS_ERROR("Invalid quaternion or missing IMU data, resetting to safe attitude");
        // 重置為默認姿態（水平）
        state_.quaternion.w() = 1.0;
        state_.quaternion.x() = 0.0;
        state_.quaternion.y() = 0.0;
        state_.quaternion.z() = 0.0;
    }
}

void PX4StyleEKF::gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // 計算時間間隔，用於確定GPS速度更新的效果
    static ros::Time last_gps_vel_time = ros::Time::now();
    double dt = (msg->header.stamp - last_gps_vel_time).toSec();
    last_gps_vel_time = msg->header.stamp;
    
    if (dt <= 0.0 || dt > 1.0) {  // 時間間隔無效或太長，直接返回
        return;
    }
    
    // 確保GPS速度在合理範圍內
    Eigen::Vector3d gps_vel(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    double vel_norm = gps_vel.norm();
    
    // 可接受GPS速度的最大值 - 適用於更高速飛行
    const double MAX_ACCEPTABLE_GPS_VEL = 30.0;  // 增加到30m/s 以適應無人機飛行速度
    
    if (vel_norm > MAX_ACCEPTABLE_GPS_VEL) {
        ROS_WARN_THROTTLE(1.0, "Received unusually high GPS velocity: %.2f m/s, limiting effects", vel_norm);
        // 不完全拒絕數據，而是限制其影響
        gps_vel *= MAX_ACCEPTABLE_GPS_VEL / vel_norm;
    }
    
    // 使用GPS速度更新速度估計
    // 進一步增加GPS速度更新增益以加快響應速度，解決速度估計不足的問題
    double velocity_gain = 0.5;  // 大幅增加GPS速度更新增益
    
    // 計算速度創新(innovation) - 觀測與預測的差異
    Eigen::Vector3d vel_innovation = gps_vel - state_.velocity;
    
    // 計算速度創新的大小
    double innovation_norm = vel_innovation.norm();
    
    // 允許更大的速度創新，以應對起飛和機動
    const double MAX_VEL_INNOVATION = 15.0;  // 允許高達15m/s的創新，更適用於無人機
    
    // 如果創新太大，限制但不完全拒絕
    if (innovation_norm > MAX_VEL_INNOVATION) {
        ROS_INFO("Large velocity innovation: %.2f m/s, limiting impact", innovation_norm);
        vel_innovation *= MAX_VEL_INNOVATION / innovation_norm;
    }
    
    // 檢測到快速或異常變化時進一步提高增益，提高速度響應性
    if (innovation_norm > 0.5) {  // 當預測與測量速度差異較大時
        velocity_gain = std::min(0.8, velocity_gain * (1.0 + innovation_norm * 0.1));
        ROS_INFO("Increasing velocity gain to %.2f due to innovation of %.2f m/s", velocity_gain, innovation_norm);
    }
    
    // 更新速度狀態
    state_.velocity(0) += velocity_gain * vel_innovation(0);
    state_.velocity(1) += velocity_gain * vel_innovation(1);
    state_.velocity(2) += velocity_gain * vel_innovation(2);
    
    // 對於非常小的速度，應用較小的閾值而不是直接設為零，避免速度估計不足
    const double MIN_VELOCITY = 0.001;  // 使用更小的閾值避免完全抑制小速度
    for (int i = 0; i < 3; i++) {
        if (std::abs(state_.velocity(i)) < MIN_VELOCITY) {
            state_.velocity(i) = 0.0;
        }
    }
    
    // 更新GPS速度有效標誌
    has_gps_vel_ = true;
    
    // 在日誌顯示GPS速度信息，以便調試
    ROS_DEBUG_THROTTLE(2.0, "GPS Vel Update: vx=%.2f, vy=%.2f, vz=%.2f, gain=%.2f, innovation=%.2f",
                      gps_vel(0), gps_vel(1), gps_vel(2), velocity_gain, innovation_norm);
                      
    // 添加更多日志以幫助診斷 - 增加頻率
    static int log_counter = 0;
    if (++log_counter % 5 == 0) {  // 每5次GPS速度更新記錄一次（原為10次）
        ROS_INFO("Current vel: vx=%.3f, vy=%.3f, vz=%.3f, GPS vel: vx=%.3f, vy=%.3f, vz=%.3f, Innovation: %.3f m/s",
                state_.velocity(0), state_.velocity(1), state_.velocity(2),
                gps_vel(0), gps_vel(1), gps_vel(2), innovation_norm);
    }
}

// 添加geographic_to_NED坐標轉換函數實現
void PX4StyleEKF::geographic_to_NED(double lat, double lon, double alt, double& north, double& east, double& down)
{
    if (!has_origin_) {
        ROS_WARN_THROTTLE(5.0, "GPS origin not set, cannot convert to NED coordinates");
        north = 0.0;
        east = 0.0;
        down = 0.0;
        return;
    }
    
    // 地球半徑（單位：米）
    const double EARTH_RADIUS = 6378137.0;
    
    // 將緯度和經度從度轉換為弧度
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double ref_lat_rad = latitude_origin_ * M_PI / 180.0;
    double ref_lon_rad = longitude_origin_ * M_PI / 180.0;
    
    // 計算緯度差（單位：弧度）
    double dlat = lat_rad - ref_lat_rad;
    // 計算經度差（單位：弧度）
    double dlon = lon_rad - ref_lon_rad;
    
    // 計算北向距離（單位：米）
    north = EARTH_RADIUS * dlat;
    
    // 計算東向距離（單位：米）
    // 根據緯度調整經度差轉換為實際距離
    east = EARTH_RADIUS * dlon * cos(ref_lat_rad);
    
    // 計算下向距離（高度差，單位：米）
    // 注意：在NED坐標系中，向下為正
    down = altitude_origin_ - alt;
    
    // 添加詳細的日誌輸出以便調試
    ROS_DEBUG_THROTTLE(10.0, "Geographic to NED: lat=%.7f, lon=%.7f, alt=%.2f -> N=%.2f, E=%.2f, D=%.2f", 
                      lat, lon, alt, north, east, down);
}

} // namespace ekf 