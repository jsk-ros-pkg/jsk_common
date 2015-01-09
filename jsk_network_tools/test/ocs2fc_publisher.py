#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import OCS2FC

if __name__ == "__main__":
    rospy.init_node("ocs2fc_test_publisher")
    pub = rospy.Publisher("ocs2fc_original", OCS2FC)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = OCS2FC()
        msg.joint_angles = [10] * 32
        msg.stop = True
        pub.publish(msg)
        r.sleep()
        
