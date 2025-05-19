#include <ros/ros.h>
#include <ekf/px4_style_ekf.h>
#include <memory>

int main(int argc, char** argv) {
    // 初始化ROS節點
    ros::init(argc, argv, "px4_style_ekf");
    
    // 顯示啟動訊息
    ROS_INFO("Starting PX4 Style EKF Node");
    
    try {
        // 創建EKF實例
        std::unique_ptr<ekf::PX4StyleEKF> px4_ekf(new ekf::PX4StyleEKF());
        
        // 進入ROS循環
        ros::spin();
        
        ROS_INFO("PX4 Style EKF Node shutting down");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in PX4 Style EKF: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown exception in PX4 Style EKF");
        return 1;
    }
    
    return 0;
} 