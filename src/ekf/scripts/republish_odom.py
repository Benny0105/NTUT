#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

first = True
h_ref = 0.0

def model_states_cb(msg):
    global first, h_ref
    # 找到 iris_vision
    try:
        i = msg.name.index('iris_vision')
    except ValueError:
        return

    # ENU 位置
    x_enu = msg.pose[i].position.x
    y_enu = msg.pose[i].position.y
    z_enu = msg.pose[i].position.z

    # 首帧设高度基准
    if first:
        h_ref = z_enu
        first = False
        rospy.loginfo("初始高度基準設置為: %.2f 米", h_ref)

    odom = Odometry()
    # 用仿真时钟
    odom.header.stamp    = rospy.Time.now()
    # 必须和 EKF launch 里 map_frame_id 一致
    odom.header.frame_id = 'map_ned'
    # 必须和 EKF launch 里 base_frame_id 一致
    odom.child_frame_id  = 'odom_ned'

    # ENU→NED 位置（北、东、下）
    odom.pose.pose.position.x =  y_enu
    odom.pose.pose.position.y =  x_enu
    odom.pose.pose.position.z = -(z_enu - h_ref)  # 使用相對高度

    # 保留原四元数
    odom.pose.pose.orientation = msg.pose[i].orientation

    # ENU→NED 线速度
    vx, vy, vz = (
        msg.twist[i].linear.x,
        msg.twist[i].linear.y,
        msg.twist[i].linear.z
    )
    odom.twist.twist.linear.x =  vy
    odom.twist.twist.linear.y =  vx
    odom.twist.twist.linear.z = -vz

    # ENU→NED 角速度
    avx, avy, avz = (
        msg.twist[i].angular.x,
        msg.twist[i].angular.y,
        msg.twist[i].angular.z
    )
    odom.twist.twist.angular.x =  avy
    odom.twist.twist.angular.y =  avx
    odom.twist.twist.angular.z = -avz

    # 添加協方差信息，特別是高度方向
    cov = [0.0]*36
    cov[0] = 0.05  # x位置方差（增加）
    cov[7] = 0.05  # y位置方差（增加）
    cov[14] = 0.1  # z位置方差（高度）- 更大的噪聲
    odom.pose.covariance = cov

    # 添加速度協方差
    vel_cov = [0.0]*36
    vel_cov[0] = 0.2   # vx速度方差
    vel_cov[7] = 0.2   # vy速度方差
    vel_cov[14] = 0.5  # vz速度方差（垂直方向較大）
    odom.twist.covariance = vel_cov

    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('gazebo_odom_republisher')
    pub = rospy.Publisher('/gazebo/odom_ned', Odometry, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb)
    rospy.spin()
