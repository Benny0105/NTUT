#!/usr/bin/env python3
import rospy
import os
import time
import math
import numpy as np
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
import cv2

# Import functionality from feature2.py
from feature2 import find_most_similar_images

class VisualGPSPublisher:
    def __init__(self):
        rospy.init_node('visual_gps_publisher', anonymous=True)
        
        # Initialize GPS publisher
        self.gps_pub = rospy.Publisher('/visual_gps/fix', NavSatFix, queue_size=10)
        self.rate = rospy.Rate(5)  # 5Hz publishing frequency, can be adjusted as needed
        
        # Set image storage paths
        self.forward_save_path = rospy.get_param('~forward_save_path', '/home/benny/catkin_ws/src/ekf/images/forward/')
        self.return_save_path = rospy.get_param('~return_save_path', '/home/benny/catkin_ws/src/ekf/images/return/')
        
        # Ensure directories exist
        os.makedirs(self.forward_save_path, exist_ok=True)
        os.makedirs(self.return_save_path, exist_ok=True)
        
        # Create image processing tool
        self.bridge = CvBridge()
        
        # Subscribe to camera image, set appropriate topic name if camera is available
        # If no camera is available, this line can be commented out and use images from folder instead
        self.image_sub = rospy.Subscriber('/iris_vision/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        
        # Current image and simulated timestamp
        self.current_image = None
        self.last_image_time = None
        self.current_image_path = None
        
        # Start GPS
        self.start_gps = None
        
        # Global storage for interpolated GPS points
        self.interpolated_gps_data = []
        
        rospy.loginfo("Visual GPS publisher initialized")
    
    def image_callback(self, msg):
        """Process subscribed camera images"""
        try:
            # Convert ROS image message to OpenCV format
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_image_time = rospy.Time.now()
            
            # Save image
            timestamp = self.last_image_time.to_sec()
            filename = f"{timestamp}.jpg"
            self.current_image_path = os.path.join(self.return_save_path, filename)
            cv2.imwrite(self.current_image_path, self.current_image)
            
            # Process image and publish GPS
            self.process_image_and_publish_gps()
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def process_image_and_publish_gps(self):
        """Process current image, find matches, and publish GPS message"""
        if not self.current_image_path or not os.path.exists(self.current_image_path):
            rospy.logwarn("No current image available")
            return
        
        try:
            # Compare with images in folder to find the two most similar images
            matches, _ = find_most_similar_images(self.current_image_path, self.forward_save_path, num_matches=2)
            
            if len(matches) < 2:
                rospy.logwarn("Not enough matching images found")
                return
            
            # Parse GPS information from matching images and calculate interpolation
            _, file_path1, *_ = matches[0]
            _, file_path2, *_ = matches[1]
            similarity1 = float(matches[0][0])
            similarity2 = float(matches[1][0])
            
            # Extract GPS coordinates from filename (assuming format is "latitude,longitude.jpg")
            try:
                parts1 = os.path.splitext(os.path.basename(file_path1))[0].split(',')
                parts2 = os.path.splitext(os.path.basename(file_path2))[0].split(',')
                
                lat1, lon1 = float(parts1[0]), float(parts1[1])
                lat2, lon2 = float(parts2[0]), float(parts2[1])
                
                # Calculate using similarity as weight
                total_similarity = similarity1 + similarity2
                if total_similarity > 0:
                    weighted_lat = (similarity1 * lat1 + similarity2 * lat2) / total_similarity
                    weighted_lon = (similarity1 * lon1 + similarity2 * lon2) / total_similarity
                else:
                    weighted_lat = (lat1 + lat2) / 2
                    weighted_lon = (lon1 + lon2) / 2
                
                # Publish GPS message
                self.publish_gps(weighted_lat, weighted_lon)
                
                # Save start GPS (if not set yet)
                if self.start_gps is None:
                    self.start_gps = (weighted_lat, weighted_lon)
                    rospy.loginfo(f"Set start GPS: ({weighted_lat:.6f}, {weighted_lon:.6f})")
                
                # Use interpolation to publish more GPS points (optional)
                self.publish_interpolated_gps(lat1, lon1, lat2, lon2, points=10)
                
            except (ValueError, IndexError) as e:
                rospy.logwarn(f"Failed to parse GPS coordinates: {e}")
                rospy.logwarn(f"File1: {file_path1}, File2: {file_path2}")
                
        except Exception as e:
            rospy.logerr(f"Error processing and publishing GPS: {e}")
    
    def publish_gps(self, lat, lon, alt=0.0):
        """Publish a single GPS message"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "map"
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        
        # Set position covariance (5 meter accuracy on diagonal)
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.position_covariance = [5.0, 0.0, 0.0, 
                                     0.0, 5.0, 0.0, 
                                     0.0, 0.0, 10.0]
        
        self.gps_pub.publish(gps_msg)
        rospy.loginfo(f"Published GPS: ({lat:.6f}, {lon:.6f})")
    
    def publish_interpolated_gps(self, lat1, lon1, lat2, lon2, points=10):
        """Perform linear interpolation between two points and publish multiple GPS points"""
        self.interpolated_gps_data = []
        
        rospy.loginfo(f"Publishing interpolated GPS from ({lat1:.6f}, {lon1:.6f}) to ({lat2:.6f}, {lon2:.6f}):")
        
        for i in range(points):
            fraction = i / (points - 1.0)  # From 0 to 1, including endpoints
            interp_lat = lat1 + fraction * (lat2 - lat1)
            interp_lon = lon1 + fraction * (lon2 - lon1)
            
            # Store interpolation point
            self.interpolated_gps_data.append((interp_lat, interp_lon))
            
            # Publish this interpolation point
            gps_msg = NavSatFix()
            gps_msg.header.stamp = rospy.Time.now()
            gps_msg.header.frame_id = "map"
            gps_msg.latitude = interp_lat
            gps_msg.longitude = interp_lon
            gps_msg.altitude = 0.0
            
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            gps_msg.position_covariance = [5.0, 0.0, 0.0, 
                                         0.0, 5.0, 0.0, 
                                         0.0, 0.0, 10.0]
            
            self.gps_pub.publish(gps_msg)
            rospy.loginfo(f"  Interpolation point {i+1}/{points}: ({interp_lat:.6f}, {interp_lon:.6f})")
            rospy.sleep(0.05)  # Short delay to avoid message congestion
    
    def simulate_from_images(self):
        """
        If no real-time camera message is available, use this method to simulate processing
        Read pre-stored images from return_save_path and process them
        """
        files = [f for f in os.listdir(self.return_save_path) if f.endswith(".jpg")]
        if not files:
            rospy.logwarn(f"No images found in {self.return_save_path}")
            return False
        
        for file in sorted(files):
            rospy.loginfo(f"Processing image: {file}")
            self.current_image_path = os.path.join(self.return_save_path, file)
            self.process_image_and_publish_gps()
            self.rate.sleep()
        
        return True
    
    def run(self):
        """Main running loop"""
        rospy.loginfo("Visual GPS publisher started running")
        
        # Check if camera subscription exists
        if self.image_sub is None:
            rospy.loginfo("No camera detected, using simulation mode")
            while not rospy.is_shutdown():
                success = self.simulate_from_images()
                if not success:
                    # If no images, publish test GPS data
                    test_lat1 = 23.973875  # Start latitude
                    test_lon1 = 120.982025  # Start longitude
                    test_lat2 = 23.974875  # End latitude
                    test_lon2 = 120.983025  # End longitude
                    rospy.loginfo("Using test GPS data:")
                    self.publish_interpolated_gps(test_lat1, test_lon1, test_lat2, test_lon2, points=20)
                
                # Cycle through publishing the last set of GPS data
                if self.interpolated_gps_data:
                    for lat, lon in self.interpolated_gps_data:
                        self.publish_gps(lat, lon)
                        self.rate.sleep()
                else:
                    self.rate.sleep()
        else:
            # If camera subscription exists, use callback for automatic processing
            while not rospy.is_shutdown():
                # Check if image is expired
                if self.last_image_time and (rospy.Time.now() - self.last_image_time).to_sec() > 5.0:
                    rospy.logwarn("Camera image has not been updated for more than 5 seconds")
                    
                    # If interpolation data exists, continue publishing last known GPS
                    if self.interpolated_gps_data:
                        for lat, lon in self.interpolated_gps_data:
                            self.publish_gps(lat, lon)
                            if rospy.is_shutdown():
                                break
                            self.rate.sleep()
                else:
                    self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = VisualGPSPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 