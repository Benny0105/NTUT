#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>

// 全局變量
double SCALE_FACTOR = 1.0;  // 默認尺度因子
std::string OUTPUT_PATH;    // 存儲調整後的位姿軌跡的路徑
std::ofstream output_file;  // 輸出文件流
ros::Publisher pub_odom;    // 發布調整尺度後的里程計
ros::Publisher pub_path;    // 發布調整尺度後的路徑
nav_msgs::Path path;        // 路徑消息

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    // 創建新的里程計消息
    nav_msgs::Odometry scaled_odom = *odom_msg;
    
    // 應用尺度因子到位置
    scaled_odom.pose.pose.position.x *= SCALE_FACTOR;
    scaled_odom.pose.pose.position.y *= SCALE_FACTOR;
    scaled_odom.pose.pose.position.z *= SCALE_FACTOR;
    
    // 發布應用尺度因子後的里程計
    pub_odom.publish(scaled_odom);
    
    // 添加到路徑
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = scaled_odom.header;
    pose_stamped.pose = scaled_odom.pose.pose;
    path.header = scaled_odom.header;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
    
    // 保存到文件
    if (output_file.is_open()) {
        output_file << scaled_odom.header.stamp.toSec() << " "
                   << scaled_odom.pose.pose.position.x << " "
                   << scaled_odom.pose.pose.position.y << " "
                   << scaled_odom.pose.pose.position.z << " "
                   << scaled_odom.pose.pose.orientation.x << " "
                   << scaled_odom.pose.pose.orientation.y << " "
                   << scaled_odom.pose.pose.orientation.z << " "
                   << scaled_odom.pose.pose.orientation.w << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vins_scale_adjuster");
    ros::NodeHandle nh("~");
    
    // 讀取參數
    nh.param("scale_factor", SCALE_FACTOR, 1.0);
    nh.param<std::string>("output_path", OUTPUT_PATH, "/home/jim/output/scaled_vio.csv");
    
    // 創建輸出文件
    output_file.open(OUTPUT_PATH);
    if (!output_file.is_open()) {
        ROS_ERROR("Failed to open output file at: %s", OUTPUT_PATH.c_str());
        return -1;
    }
    
    ROS_INFO("VINS Scale Adjuster Node started with scale factor: %f", SCALE_FACTOR);
    ROS_INFO("Output will be saved to: %s", OUTPUT_PATH.c_str());
    
    // 訂閱VINS-Fusion的odometry輸出
    ros::Subscriber sub_odom = nh.subscribe("/vins/odometry", 2000, odomCallback);
    
    // 發布調整尺度後的里程計和路徑
    pub_odom = nh.advertise<nav_msgs::Odometry>("/vins/scaled_odometry", 2000);
    pub_path = nh.advertise<nav_msgs::Path>("/vins/scaled_path", 1000);
    
    // 初始化路徑消息
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    
    ros::spin();
    
    // 關閉輸出文件
    if (output_file.is_open()) {
        output_file.close();
    }
    
    return 0;
} 