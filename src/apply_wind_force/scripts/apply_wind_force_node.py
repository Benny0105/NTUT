#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Header
from datetime import datetime, timedelta

class ApplyWindForceNode:
    def __init__(self):
        rospy.init_node('apply_wind_force_node', anonymous=True)
        
        # 等待 apply_body_wrench service 啟動
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        rospy.Subscriber('/gazebo/custom_world/world_wind', WrenchStamped, self.wind_callback)
        
        rospy.spin()

    def wind_callback(self, msg):
        rospy.loginfo(f"[ApplyWindForceNode] Received wind force: fx={msg.wrench.force.x:.2f}, fy={msg.wrench.force.y:.2f}, fz={msg.wrench.force.z:.2f}")
        
        try:
            self.apply_wrench_service(
                body_name="iris_vision::iris::base_link",
                reference_frame="world",
                reference_point=Point(0, 0, 0),
                wrench=Wrench(
                    force=msg.wrench.force,
                    torque=msg.wrench.torque
                ),
                start_time=rospy.Time(0, 0),
                duration=rospy.Duration(0.2)
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    ApplyWindForceNode()
