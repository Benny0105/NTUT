#!/usr/bin/env python3
import rospy
import rostopic
import time
from std_msgs.msg import String

def get_topic_info(topic_name):
    """Get information about publishers, subscribers and message type for a topic"""
    try:
        # Get publisher count
        info_text = rostopic.get_info_text(topic_name)
        publishers = info_text.count('* ')
        
        # Get topic type
        topic_type = rostopic.get_topic_type(topic_name)[0]
        
        # Get message class
        msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic_name)
        
        return {
            'name': topic_name,
            'type': topic_type,
            'publishers': publishers,
            'message_class': msg_class.__name__ if msg_class else 'None'
        }
    except Exception as e:
        # If we can't get information, return a structured error
        return {
            'name': topic_name,
            'error': str(e)
        }

def main():
    rospy.init_node('topic_checker', anonymous=True)
    
    # Get check frequency parameter
    check_frequency = rospy.get_param('~check_frequency', 1.0)
    
    # Create publisher for results
    results_pub = rospy.Publisher('/topic_checker/results', String, queue_size=10)
    
    # List of topics to check - aligned with the current setup
    topics_to_check = [
        # PX4/MAVROS topics
        '/mavros/imu/data_raw',
        '/mavros/global_position/global',
        '/mavros/imu/mag',
        '/mavros/imu/static_pressure',
        
        # Barometer conversion topics
        '/baro/pressure',
        '/baro/height',
        
        # EKF input topics
        '/imu/data',
        '/gps/fix',
        '/imu/mag',
        
        # Visual GPS and output topics
        '/visual_gps/fix',
        '/px4_style_ekf/pose',
        '/px4_style_ekf/velocity',
        '/px4_style_ekf/odom'
    ]
    
    rate = rospy.Rate(check_frequency)
    
    while not rospy.is_shutdown():
        results = []
        
        # Add timestamp to results
        timestamp = rospy.get_time()
        results.append(f"Topic Check Time: {timestamp:.2f}")
        
        for topic in topics_to_check:
            info = get_topic_info(topic)
            
            if 'error' in info:
                rospy.logwarn(f"Cannot get information for topic {topic}: {info['error']}")
                # Don't spam the log with the same warnings repeatedly
                continue
            
            status = f"Topic: {topic}\n  Type: {info['type']}\n  Publishers: {info['publishers']}\n  Message class: {info['message_class']}\n"
            results.append(status)
            rospy.loginfo(status)
        
        # Add empty line for readability in logs
        rospy.loginfo("----------------------------------------")
        
        # Publish results
        results_pub.publish('\n'.join(results))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 