#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
import random

def random_wind_publisher():
    rospy.init_node('random_wind_publisher', anonymous=True)
    pub = rospy.Publisher('/gazebo/custom_world/world_wind', WrenchStamped, queue_size=10)
    rate = rospy.Rate(10)  # 發布頻率 10 Hz

    # 風力範圍設定（可自由調整）
    force_x_range = (-4.0, 4.0)
    force_y_range = (-4.0, 4.0)
    force_z_value = 0.0  # 風吹不往上推

    while not rospy.is_shutdown():
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        # 隨機生成風力
        msg.wrench.force.x = random.uniform(*force_x_range)
        msg.wrench.force.y = random.uniform(*force_y_range)
        msg.wrench.force.z = force_z_value

        # 沒有旋轉力矩，全部設成0
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        random_wind_publisher()
    except rospy.ROSInterruptException:
        pass
