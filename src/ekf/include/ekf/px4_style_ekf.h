#ifndef PX4_STYLE_EKF_H
#define PX4_STYLE_EKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <deque>
#include <memory>

namespace ekf {

// 融合狀態結構體，用於跟蹤各傳感器的融合狀態
struct FusionStatus {
    bool gps_fusion_active;       // GPS位置融合活動
    bool baro_fusion_active;      // 氣壓計高度融合活動
    bool mag_fusion_active;       // 磁力計融合活動
    bool terrain_fusion_active;   // 地形高度融合活動
    bool airspeed_fusion_active;  // 空速融合活動
    bool wind_estimation_active;  // 風速估計活動
    
    FusionStatus() : 
        gps_fusion_active(false),
        baro_fusion_active(false),
        mag_fusion_active(false),
        terrain_fusion_active(false),
        airspeed_fusion_active(false),
        wind_estimation_active(false) {}
};

// 添加PX4風格EKF計算所需的結構體和常量
struct EkfState {
    // 位置 (NED座標系)
    Eigen::Vector3d position;
    
    // 速度 (NED座標系)
    Eigen::Vector3d velocity;
    
    // 姿態四元數 (NED座標系中的本體到參考系旋轉)
    Eigen::Quaterniond quaternion;
    
    // IMU加速度偏差
    Eigen::Vector3d accel_bias;
    
    // IMU角速度偏差
    Eigen::Vector3d gyro_bias;
    
    // 位置和速度協方差
    Eigen::Matrix<double, 6, 6> pos_vel_cov;
    
    // 姿態協方差
    Eigen::Matrix3d att_cov;
    
    // 偏置協方差
    Eigen::Matrix<double, 6, 6> bias_cov;
};

// 傳感器延遲補償結構體
struct SensorDelayCompensation {
    bool enabled;
    double imu_delay;
    double gps_delay;
    double mag_delay;
    double baro_delay;
    
    // 傳感器緩衝區
    std::deque<sensor_msgs::Imu> imu_buffer;
    std::deque<sensor_msgs::NavSatFix> gps_buffer;
    std::deque<sensor_msgs::MagneticField> mag_buffer;
    std::deque<std_msgs::Float32> baro_buffer;
};

// EKF參數結構體
struct EkfParams {
    // 流程噪聲參數
    double q_pos;
    double q_vel;
    double q_att;
    double q_gyro_bias;
    double q_accel_bias;
    
    // 測量噪聲參數
    double r_gps_xy;
    double r_gps_z;
    double r_gps_vxy;
    double r_gps_vz;
    double r_mag;
    double r_baro;
    
    // 初始協方差
    double p0_pos;
    double p0_vel;
    double p0_att;
    double p0_gyro_bias;
    double p0_accel_bias;
    
    // 傳感器校準參數
    Eigen::Vector3d mag_declination;
    Eigen::Vector3d mag_scale_factors;
    
    // 傳感器檢查參數
    double gps_check_threshold;
    double mag_check_threshold;
    double baro_check_threshold;
    
    // 中值濾波器大小
    int gps_median_filter_size;
    int baro_median_filter_size;
};

class PX4StyleEKF {
public:
    PX4StyleEKF();
    virtual ~PX4StyleEKF();

    // 初始化EKF
    void initialize();

    // 添加高級EKF功能的新方法
    void setDelayCompensation(bool enabled, double imu_delay, double gps_delay, 
                             double mag_delay, double baro_delay);
    void setComplementaryFilterParams(double tau_vel, double tau_pos);
    void loadAdvancedParams(const ros::NodeHandle& nh);

private:
    // 節點句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // 訂閱者
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber baro_sub_;
    ros::Subscriber mag_sub_;
    ros::Subscriber gps_vel_sub_;  // 添加GPS速度訂閱者
    
    // 發布者
    ros::Publisher pose_pub_;
    ros::Publisher pose_stamped_pub_;  // 添加一個PoseStamped發布者
    ros::Publisher odom_pub_;
    ros::Publisher velocity_pub_;  // 添加速度發布者
    
    // 計時器
    ros::Timer update_timer_;
    
    // TF廣播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 基本參數
    double freq_;                 // EKF更新頻率(Hz)
    double sensor_timeout_;       // 傳感器數據超時時間(秒)
    std::string base_frame_id_;   // 機體座標系ID
    std::string global_frame_id_; // 全局座標系ID
    std::string output_frame_id_; // 輸出座標系ID
    
    // 互補濾波器時間常數
    double tau_vel_;              // 速度濾波器時間常數
    double tau_pos_;              // 位置濾波器時間常數
    
    // 計算的最大延遲
    double max_delay_;            // 最大傳感器延遲
    
    // 傳感器設置
    bool use_imu_;               // 使用IMU數據
    bool use_gps_;               // 使用GPS數據
    bool use_mag_;               // 使用磁力計數據
    bool use_baro_;              // 使用氣壓計數據
    bool use_odom_;              // 使用里程計數據
    bool publish_tf_;            // 發布TF變換
    
    // 延遲補償參數(秒)
    double imu_delay_;           // IMU數據延遲
    double gps_delay_;           // GPS數據延遲
    double mag_delay_;           // 磁力計數據延遲
    double baro_delay_;          // 氣壓計數據延遲
    double odom_delay_;          // 里程計數據延遲
    
    // 噪聲參數
    double gyro_noise_;          // 陀螺儀噪聲(rad/s)
    double accel_noise_;         // 加速度計噪聲(m/s^2)
    double gps_pos_noise_x_;     // GPS位置噪聲X(m)
    double gps_pos_noise_y_;     // GPS位置噪聲Y(m)
    double gps_pos_noise_z_;     // GPS位置噪聲Z(m)
    double gps_vel_noise_;       // GPS速度噪聲(m/s)
    double mag_noise_;           // 磁力計噪聲(gauss)
    double baro_noise_;          // 氣壓計高度噪聲(m)
    
    // 創新門限參數
    double pos_innov_gate_;      // 位置更新創新門限
    double vel_innov_gate_;      // 速度更新創新門限
    double hgt_innov_gate_;      // 高度更新創新門限
    double mag_innov_gate_;      // 磁力計更新創新門限
    
    // 傳感器控制參數(位掩碼)
    int imu_control_;            // IMU融合控制
    int gps_control_;            // GPS融合控制
    int baro_control_;           // 氣壓計融合控制
    int mag_control_;            // 磁力計融合控制
    
    // 高度傳感器選擇
    int height_sensor_;          // 高度數據來源(0=氣壓計,1=GPS,2=測距儀,3=視覺)
    
    // 座標系統設置
    std::string coordinate_system_;  // 座標系統類型 (ned 或 enu)
    
    // EKF狀態
    EkfState state_;
    
    // 協方差矩陣
    Eigen::MatrixXd P_;
    
    // 參數配置
    EkfParams params_;
    
    // 延遲補償
    SensorDelayCompensation delay_comp_;
    
    // 初始化標誌
    bool initialized_;           // EKF是否已初始化
    bool gps_origin_initialized_; // GPS原點是否已初始化
    bool first_update_;          // 第一次更新標誌
    
    // 時間戳
    ros::Time last_update_time_; // 上次更新時間
    ros::Time last_imu_time_;    // 上次IMU數據時間
    ros::Time last_gps_time_;    // 上次GPS數據時間
    ros::Time last_mag_time_;    // 上次磁力計數據時間
    ros::Time last_baro_time_;   // 上次氣壓計數據時間
    
    // 最新的傳感器數據
    sensor_msgs::Imu::ConstPtr last_imu_msg_;    // 最新的IMU消息
    
    // GPS原點
    double gps_origin_lat_;      // GPS原點緯度
    double gps_origin_lon_;      // GPS原點經度
    double gps_origin_alt_;      // GPS原點高度
    
    // 新的GPS原點變量
    double latitude_origin_;     // 緯度原點
    double longitude_origin_;    // 經度原點
    double altitude_origin_;     // 高度原點
    bool has_origin_;            // 是否已設置原點
    bool has_gps_;               // 是否有GPS數據
    bool has_gps_vel_;           // 是否有GPS速度數據
    double baro_ref_alt_;        // 氣壓計參考高度
    bool has_baro_ref_;          // 是否有氣壓計參考數據
    double local_n, local_e, local_d;  // 本地NED坐標
    
    // 氣壓計基準
    double baro_origin_height_;  // 氣壓計原始高度
    bool first_baro_received_;   // 是否收到第一個氣壓計數據
    
    // 融合狀態
    FusionStatus fusion_status_; // 當前融合狀態
    
    // 處理傳感器數據的回調函數
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void baroCallback(const std_msgs::Float32::ConstPtr& msg);
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
    void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);  // 添加GPS速度回調
    void updateTimerCallback(const ros::TimerEvent& event);
    
    // 更新函數
    void updateWithIMU(const sensor_msgs::Imu::ConstPtr& msg);
    void updateWithGPS(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void updateWithBaro(const std_msgs::Float32::ConstPtr& msg);
    void updateWithMag(const sensor_msgs::MagneticField::ConstPtr& msg);
    void predict(const ros::Time& current_time);
    
    // 公布當前狀態
    void publishState();
    
    // 坐標轉換工具函數
    void convertLLAtoNED(double lat, double lon, double alt, double& north, double& east, double& down);
    void geographic_to_NED(double lat, double lon, double alt, double& north, double& east, double& down);  // 添加新的坐標轉換函數
    double getEarthRadius();
    
    // 檢查傳感器數據有效性的方法
    bool checkGpsValidity(const sensor_msgs::NavSatFix& gps_msg);
    bool checkMagValidity(const sensor_msgs::MagneticField& mag_msg);
    bool checkBaroValidity(const std_msgs::Float32& baro_msg);
    
    // 新增的輔助函數
    double normalizeAngle(double angle);
    void resetAttitudeIfNeeded();
    
    // 傳感器融合方法
    void fuseGps(const sensor_msgs::NavSatFix& gps_msg);
    void fuseMag(const sensor_msgs::MagneticField& mag_msg);
    void fuseBaro(const std_msgs::Float32& baro_msg);
    
    // 互補濾波器更新方法
    void complementaryFilterUpdate();
    
    // 協方差預測和更新方法
    void predictCovariance(double dt);
    void updateCovarianceWithGps();
    void updateCovarianceWithMag();
    void updateCovarianceWithBaro();
    
    // 數據緩衝區管理方法
    void pruneBuffers(const ros::Time& current_time);
    void getSyncedSensorData(const ros::Time& target_time, 
                            sensor_msgs::Imu& imu_out,
                            sensor_msgs::NavSatFix& gps_out,
                            sensor_msgs::MagneticField& mag_out,
                            std_msgs::Float32& baro_out);
                            
    // 座標轉換方法
    Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned_vector);
    Eigen::Quaterniond nedToEnuQuaternion(const Eigen::Quaterniond& ned_quat);
};

} // namespace ekf

#endif // PX4_STYLE_EKF_H 