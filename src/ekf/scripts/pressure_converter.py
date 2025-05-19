#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32

class PressureConverter:
    def __init__(self):
        # Initialize node
        rospy.init_node('pressure_converter', anonymous=True)
        
        # Get parameters
        self.input_topic = rospy.get_param('~input_topic', '/mavros/imu/static_pressure')
        self.output_topic = rospy.get_param('~output_topic', '/baro/pressure')
        
        # Create publishers
        self.pressure_pub = rospy.Publisher(self.output_topic, FluidPressure, queue_size=10)
        self.height_pub = rospy.Publisher('/baro/height', Float32, queue_size=10)
        
        # Create subscriber
        self.pressure_sub = rospy.Subscriber(self.input_topic, FluidPressure, self.pressure_callback)
        
        # Sea level pressure in hPa
        self.sea_level_pressure = 1013.25
        
        rospy.loginfo(f"Barometer data converter started")
        rospy.loginfo(f"Subscribing to: {self.input_topic} (FluidPressure)")
        rospy.loginfo(f"Publishing to: {self.output_topic} (FluidPressure)")
    
    def pressure_callback(self, msg):
        """Process FluidPressure message and convert to height"""
        # Forward the same FluidPressure message
        self.pressure_pub.publish(msg)
        
        # Also convert to height and publish
        height_msg = Float32()
        
        # Pressure altitude formula: h = 44330 * (1 - (p/p0)^0.19029)
        pressure_hpa = msg.fluid_pressure / 100.0  # Convert Pa to hPa
        altitude = 44330.0 * (1.0 - pow(pressure_hpa / self.sea_level_pressure, 0.19029))
        
        height_msg.data = altitude
        self.height_pub.publish(height_msg)

if __name__ == '__main__':
    try:
        converter = PressureConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 