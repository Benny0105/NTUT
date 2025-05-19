#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/tf.h>

class PX4GazeboInterface {
public:
    PX4GazeboInterface() : nh_(""), private_nh_("~") {
        // Get parameters
        private_nh_.param<std::string>("px4_imu_topic", px4_imu_topic_, "/mavros/imu/data_raw");
        private_nh_.param<std::string>("px4_gps_topic", px4_gps_topic_, "/mavros/global_position/global");
        private_nh_.param<std::string>("px4_mag_topic", px4_mag_topic_, "/mavros/imu/mag");
        private_nh_.param<std::string>("px4_baro_topic", px4_baro_topic_, "/mavros/imu/static_pressure");
        
        // Get output topic parameters
        private_nh_.param<std::string>("imu_out_topic", imu_out_topic_, "/imu/data");
        private_nh_.param<std::string>("gps_out_topic", gps_out_topic_, "/gps/fix");
        private_nh_.param<std::string>("mag_out_topic", mag_out_topic_, "/imu/mag");
        private_nh_.param<std::string>("baro_out_topic", baro_out_topic_, "/baro/height");
        
        private_nh_.param<bool>("enable_position_print", enable_position_print_, true);
        private_nh_.param<double>("print_frequency", print_frequency_, 1.0);
        private_nh_.param<bool>("debug_mode", debug_mode_, false);
        
        // Get coordinate system parameters
        private_nh_.param<bool>("use_ned_frame", use_ned_frame_, true);
        private_nh_.param<bool>("convert_from_enu", convert_from_enu_, true);
        
        // Initialize subscribers
        imu_sub_ = nh_.subscribe(px4_imu_topic_, 10, &PX4GazeboInterface::imuCallback, this);
        gps_sub_ = nh_.subscribe(px4_gps_topic_, 10, &PX4GazeboInterface::gpsCallback, this);
        mag_sub_ = nh_.subscribe(px4_mag_topic_, 10, &PX4GazeboInterface::magCallback, this);
        baro_sub_ = nh_.subscribe(px4_baro_topic_, 10, &PX4GazeboInterface::baroCallback, this);
        
        // Initialize EKF output monitoring
        pose_sub_ = nh_.subscribe("/px4_style_ekf/pose", 10, &PX4GazeboInterface::poseCallback, this);
        odom_sub_ = nh_.subscribe("/px4_style_ekf/odom", 10, &PX4GazeboInterface::odomCallback, this);
        
        // Initialize publishers
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_out_topic_, 10);
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(gps_out_topic_, 10);
        mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>(mag_out_topic_, 10);
        baro_pub_ = nh_.advertise<std_msgs::Float32>(baro_out_topic_, 10);
        
        // Initialize timer for position printing
        if (enable_position_print_) {
            print_timer_ = nh_.createTimer(ros::Duration(1.0 / print_frequency_), 
                                          &PX4GazeboInterface::printTimerCallback, this);
        }
        
        // Sea level pressure (hPa)
        sea_level_pressure_ = 1013.25;
        
        // Keep track of message counts
        imu_count_ = 0;
        gps_count_ = 0;
        mag_count_ = 0;
        baro_count_ = 0;
        
        // Last data timestamps
        last_imu_time_ = ros::Time::now();
        last_gps_time_ = ros::Time::now();
        last_mag_time_ = ros::Time::now();
        last_baro_time_ = ros::Time::now();
        
        ROS_INFO("=== PX4 Gazebo interface node startup ===");
        ROS_INFO("Sea level pressure (hPa): %.2f", sea_level_pressure_);
        ROS_INFO("IMU topic: %s -> %s", px4_imu_topic_.c_str(), imu_out_topic_.c_str());
        ROS_INFO("GPS topic: %s -> %s", px4_gps_topic_.c_str(), gps_out_topic_.c_str());
        ROS_INFO("Magnetometer topic: %s -> %s", px4_mag_topic_.c_str(), mag_out_topic_.c_str());
        ROS_INFO("Barometer topic: %s -> %s", px4_baro_topic_.c_str(), baro_out_topic_.c_str());
        ROS_INFO("Coordinate system: %s (conversion from ENU: %s)", 
                use_ned_frame_ ? "NED" : "ENU", 
                convert_from_enu_ ? "enabled" : "disabled");
        if (enable_position_print_) {
            ROS_INFO("Position print: enabled (frequency: %.1f Hz)", print_frequency_);
        }
        if (debug_mode_) {
            ROS_INFO("Debug mode: enabled (will print message counts)");
        }
        ROS_INFO("Waiting for data...");
        
        // Set up status timer
        status_timer_ = nh_.createTimer(ros::Duration(5.0), &PX4GazeboInterface::statusTimerCallback, this);
    }
    
    ~PX4GazeboInterface() {}
    
private:
    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscription topic names
    std::string px4_imu_topic_;
    std::string px4_gps_topic_;
    std::string px4_mag_topic_;
    std::string px4_baro_topic_;
    
    // Publishing topic names
    std::string imu_out_topic_;
    std::string gps_out_topic_;
    std::string mag_out_topic_;
    std::string baro_out_topic_;
    
    // Subscribers
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber mag_sub_;
    ros::Subscriber baro_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    
    // Publishers
    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher baro_pub_;
    
    // Timers
    ros::Timer print_timer_;
    ros::Timer status_timer_;
    
    // Parameters
    bool enable_position_print_;
    double print_frequency_;
    double sea_level_pressure_;
    bool debug_mode_;
    bool use_ned_frame_;
    bool convert_from_enu_;
    
    // Last data timestamps
    ros::Time last_imu_time_;
    ros::Time last_gps_time_;
    ros::Time last_mag_time_;
    ros::Time last_baro_time_;
    
    // Message counts
    int imu_count_;
    int gps_count_;
    int mag_count_;
    int baro_count_;
    
    // Latest messages
    sensor_msgs::Imu latest_imu_;
    sensor_msgs::NavSatFix latest_gps_;
    sensor_msgs::MagneticField latest_mag_;
    sensor_msgs::FluidPressure latest_pressure_;
    std_msgs::Float32 latest_baro_height_;
    
    // Latest EKF output
    geometry_msgs::PoseWithCovarianceStamped latest_pose_;
    nav_msgs::Odometry latest_odom_;
    
    // Status timer callback to report health
    void statusTimerCallback(const ros::TimerEvent& event) {
        ROS_INFO("=== PX4 Gazebo Interface Status ===");
        ROS_INFO("Messages received since last status:");
        ROS_INFO("  IMU: %d", imu_count_);
        ROS_INFO("  GPS: %d", gps_count_);
        ROS_INFO("  MAG: %d", mag_count_);
        ROS_INFO("  BARO: %d", baro_count_);
        
        // Reset counts
        imu_count_ = 0;
        gps_count_ = 0;
        mag_count_ = 0;
        baro_count_ = 0;
        
        // Print current sensor health
        ROS_INFO("Sensor Status - IMU: %s, GPS: %s, MAG: %s, BARO: %s",
                 imu_count_ > 0 ? "Active" : "Inactive",
                 gps_count_ > 0 ? "Active" : "Inactive",
                 mag_count_ > 0 ? "Active" : "Inactive",
                 baro_count_ > 0 ? "Active" : "Inactive");
    }
    
    // Sensor callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (convert_from_enu_ && use_ned_frame_) {
            // Convert from ENU to NED
            sensor_msgs::Imu ned_msg = *msg;
            
            // Set header
            ned_msg.header = msg->header;
            
            // Convert orientation
            convertQuaternionENUtoNED(msg->orientation, ned_msg.orientation);
            
            // Convert angular velocity
            convertENUtoNED(msg->angular_velocity, ned_msg.angular_velocity);
            
            // Convert linear acceleration
            convertENUtoNED(msg->linear_acceleration, ned_msg.linear_acceleration);
            
            // Store and publish
            latest_imu_ = ned_msg;
            imu_pub_.publish(ned_msg);
        } else {
            // Store the message without conversion
            latest_imu_ = *msg;
            
            // Republish
            imu_pub_.publish(msg);
        }
        
        // Update timestamp
        last_imu_time_ = ros::Time::now();
        
        // Increment count
        imu_count_++;
        
        if (debug_mode_ && imu_count_ % 50 == 0) {
            ROS_INFO("Forwarded IMU message #%d", imu_count_);
        }
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // GPS coordinates don't need ENU-NED conversion (lat/lon/alt are the same)
        
        // Store the message
        latest_gps_ = *msg;
        
        // Republish
        gps_pub_.publish(msg);
        
        // Update timestamp
        last_gps_time_ = ros::Time::now();
        
        // Increment count
        gps_count_++;
        
        if (debug_mode_ && gps_count_ % 10 == 0) {
            ROS_INFO("Forwarded GPS message #%d: Lat=%.6f, Lon=%.6f", 
                     gps_count_, msg->latitude, msg->longitude);
        }
    }
    
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
        if (convert_from_enu_ && use_ned_frame_) {
            // Convert from ENU to NED
            sensor_msgs::MagneticField ned_msg = *msg;
            
            // Set header
            ned_msg.header = msg->header;
            
            // Convert magnetic field vector
            convertENUtoNED(msg->magnetic_field, ned_msg.magnetic_field);
            
            // Store and publish
            latest_mag_ = ned_msg;
            mag_pub_.publish(ned_msg);
        } else {
            // Store the message without conversion
            latest_mag_ = *msg;
            
            // Republish
            mag_pub_.publish(msg);
        }
        
        // Update timestamp
        last_mag_time_ = ros::Time::now();
        
        // Increment count
        mag_count_++;
        
        if (debug_mode_ && mag_count_ % 50 == 0) {
            ROS_INFO("Forwarded MAG message #%d", mag_count_);
        }
    }
    
    void baroCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
        // Store the raw pressure
        latest_pressure_ = *msg;
        
        // Convert pressure to height (simple model)
        std_msgs::Float32 height_msg;
        height_msg.data = pressureToHeight(msg->fluid_pressure);
        latest_baro_height_ = height_msg;
        
        // Publish height
        baro_pub_.publish(height_msg);
        
        // Update timestamp
        last_baro_time_ = ros::Time::now();
        
        // Increment count
        baro_count_++;
        
        if (debug_mode_ && baro_count_ % 50 == 0) {
            ROS_INFO("Forwarded BARO message #%d: Pressure=%.2f hPa -> Height=%.2f m", 
                     baro_count_, msg->fluid_pressure/100.0, height_msg.data);
        }
    }
    
    // EKF output callbacks
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        latest_pose_ = *msg;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        latest_odom_ = *msg;
    }
    
    // Timer callback for position printing
    void printTimerCallback(const ros::TimerEvent& event) {
        // Only print if we have received data from the EKF
        if (latest_pose_.header.stamp != ros::Time(0)) {
            printPose(latest_pose_);
        }
        
        // Check for delays in sensor data
        ros::Time now = ros::Time::now();
        double imu_delay = (now - last_imu_time_).toSec();
        double gps_delay = (now - last_gps_time_).toSec();
        double mag_delay = (now - last_mag_time_).toSec();
        double baro_delay = (now - last_baro_time_).toSec();
        
        // Only print warnings if data is actually being received and delays are excessive
        if (imu_count_ > 0 && imu_delay > 1.0) {
            ROS_INFO("IMU data delay: %.1f seconds", imu_delay);
        }
        if (gps_count_ > 0 && gps_delay > 1.0) {
            ROS_INFO("GPS data delay: %.1f seconds", gps_delay);
        }
        if (mag_count_ > 0 && mag_delay > 1.0) {
            ROS_INFO("Magnetometer data delay: %.1f seconds", mag_delay);
        }
        if (baro_count_ > 0 && baro_delay > 1.0) {
            ROS_INFO("Barometer data delay: %.1f seconds", baro_delay);
        }
    }
    
    // Convert pressure to height
    float pressureToHeight(float pressure_pa) {
        // Formula: h = 44330 * (1 - (p/p0)^(1/5.255))
        // where p0 is sea level pressure (1013.25 hPa)
        float pressure_hpa = pressure_pa / 100.0f;  // Convert Pa to hPa
        float ratio = pressure_hpa / sea_level_pressure_;
        float height = 44330.0f * (1.0f - pow(ratio, 0.190295f));
        return height;
    }
    
    // Print position data
    void printPose(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        // Extract data
        double x = pose.pose.pose.position.x;
        double y = pose.pose.pose.position.y;
        double z = pose.pose.pose.position.z;
        
        double qx = pose.pose.pose.orientation.x;
        double qy = pose.pose.pose.orientation.y;
        double qz = pose.pose.pose.orientation.z;
        double qw = pose.pose.pose.orientation.w;
        
        // Print
        if (use_ned_frame_) {
            ROS_INFO("Position (NED, m): North=%.2f East=%.2f Down=%.2f", x, y, z);
        } else {
            ROS_INFO("Position (ENU, m): X=%.2f Y=%.2f Z=%.2f", x, y, z);
        }
        ROS_INFO("Orientation (quaternion): X=%.4f Y=%.4f Z=%.4f W=%.4f", qx, qy, qz, qw);
    }

    // 添加ENU到NED座標轉換函數
    void convertENUtoNED(const geometry_msgs::Vector3& enu, geometry_msgs::Vector3& ned)
    {
        // ENU(東北上) 到 NED(北東下)的轉換
        ned.x = enu.y;   // ENU.y (North) -> NED.x (North)
        ned.y = enu.x;   // ENU.x (East) -> NED.y (East)
        ned.z = -enu.z;  // ENU.z (Up) -> NED.z (Down)
    }

    // 添加四元數轉換函數：從ENU到NED
    void convertQuaternionENUtoNED(const geometry_msgs::Quaternion& enu_quat, geometry_msgs::Quaternion& ned_quat)
    {
        // 創建旋轉矩陣，將ENU到NED的旋轉
        tf::Quaternion q_enu_to_ned;
        q_enu_to_ned.setRPY(M_PI/2, 0, M_PI/2);
        
        // 轉換輸入的四元數
        tf::Quaternion q_enu;
        tf::quaternionMsgToTF(enu_quat, q_enu);
        
        // 應用轉換
        tf::Quaternion q_ned = q_enu_to_ned * q_enu * q_enu_to_ned.inverse();
        
        // 轉回消息格式
        tf::quaternionTFToMsg(q_ned, ned_quat);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_gazebo_interface");
    
    PX4GazeboInterface interface;
    
    ros::spin();
    
    return 0;
} 