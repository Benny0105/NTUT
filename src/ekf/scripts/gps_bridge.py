#!/usr/bin/env python3
# gps_bridge.py - 將原始 GPS 數據轉發到全球位置話題

import rospy
from sensor_msgs.msg import NavSatFix

class GPSBridge:
    def __init__(self):
        rospy.init_node('gps_bridge', anonymous=True)
        
        # 獲取參數，如果沒有設置，使用默認值
        self.input_topic = rospy.get_param('~input_topic', '/mavros/global_position/raw/fix')
        self.output_topic = rospy.get_param('~output_topic', '/mavros/global_position/global')
        self.debug = rospy.get_param('~debug', True)
        
        # 設置訂閱和發布
        self.gps_sub = rospy.Subscriber(self.input_topic, NavSatFix, self.gps_callback)
        self.gps_pub = rospy.Publisher(self.output_topic, NavSatFix, queue_size=10)
        
        rospy.loginfo(f"GPS 橋接節點啟動!")
        rospy.loginfo(f"將 {self.input_topic} 轉發到 {self.output_topic}")
        
        # 追蹤發布的消息數量
        self.msg_count = 0
        self.last_log_time = rospy.Time.now()
        
    def gps_callback(self, msg):
        # 更新消息的時間戳為當前時間
        msg.header.stamp = rospy.Time.now()
        
        # 發布消息
        self.gps_pub.publish(msg)
        
        # 計數器增加
        self.msg_count += 1
        
        # 每秒打印一次狀態
        if self.debug and (rospy.Time.now() - self.last_log_time).to_sec() >= 1.0:
            rospy.loginfo(f"已轉發 {self.msg_count} 條 GPS 消息")
            self.last_log_time = rospy.Time.now()

if __name__ == '__main__':
    try:
        bridge = GPSBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 