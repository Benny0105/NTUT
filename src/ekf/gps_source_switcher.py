#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool, SetBoolResponse

class GPSSourceSwitcher:
    def __init__(self):
        rospy.init_node('gps_source_switcher', anonymous=True)
        
        # Get parameters
        self.rate = rospy.Rate(rospy.get_param('~rate', 10))  # Default 10Hz
        
        # GPS source and target topics
        self.px4_gps_topic = rospy.get_param('~px4_gps_topic', '/mavros/global_position/global')
        self.visual_gps_topic = rospy.get_param('~visual_gps_topic', '/visual_gps/fix')
        self.output_gps_topic = rospy.get_param('~output_gps_topic', '/ekf/gps')
        
        # Create publishers and subscribers
        self.gps_pub = rospy.Publisher(self.output_gps_topic, NavSatFix, queue_size=10)
        self.px4_gps_sub = rospy.Subscriber(self.px4_gps_topic, NavSatFix, self.px4_gps_callback)
        self.visual_gps_sub = rospy.Subscriber(self.visual_gps_topic, NavSatFix, self.visual_gps_callback)
        
        # Status control
        self.use_visual_gps = False  # Default use PX4 GPS
        self.last_px4_gps = None
        self.last_visual_gps = None
        
        # Set up service to switch GPS source
        self.switch_service = rospy.Service('~switch_to_visual_gps', SetBool, self.switch_gps_source)
        
        rospy.loginfo(f"GPS source switcher initialization complete")
        rospy.loginfo(f"Current GPS source: {'Visual GPS' if self.use_visual_gps else 'PX4 GPS'}")
        rospy.loginfo(f"Output topic: {self.output_gps_topic}")
    
    def px4_gps_callback(self, msg):
        """Process PX4 GPS message"""
        self.last_px4_gps = msg
        
        # If set to use PX4 GPS, forward the message
        if not self.use_visual_gps:
            self.gps_pub.publish(msg)
    
    def visual_gps_callback(self, msg):
        """Process visual GPS message"""
        self.last_visual_gps = msg
        
        # If set to use visual GPS, forward the message
        if self.use_visual_gps:
            self.gps_pub.publish(msg)
    
    def switch_gps_source(self, req):
        """GPS source switching service handler"""
        self.use_visual_gps = req.data
        if self.use_visual_gps:
            source = "Visual GPS"
            # Immediately publish the last visual GPS message (if available)
            if self.last_visual_gps:
                self.gps_pub.publish(self.last_visual_gps)
        else:
            source = "PX4 GPS"
            # Immediately publish the last PX4 GPS message (if available)
            if self.last_px4_gps:
                self.gps_pub.publish(self.last_px4_gps)
        
        rospy.loginfo(f"Switched GPS source to: {source}")
        return SetBoolResponse(success=True, message=f"Switched GPS source to: {source}")
    
    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # Ensure GPS messages are published (republish last received message when no new messages)
            if self.use_visual_gps and self.last_visual_gps:
                self.gps_pub.publish(self.last_visual_gps)
            elif not self.use_visual_gps and self.last_px4_gps:
                self.gps_pub.publish(self.last_px4_gps)
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gps_switcher = GPSSourceSwitcher()
        gps_switcher.run()
    except rospy.ROSInterruptException:
        pass 