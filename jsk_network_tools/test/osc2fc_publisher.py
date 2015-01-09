#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import OSC2FC

if __name__ == "__main__":
    rospy.init_node("osc2fc_test_publisher")
    pub = rospy.Publisher("osc2fc_original", OSC2FC)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = OSC2FC()
        msg.joint_angles = [10] * 32
        msg.stop = True
        pub.publish(msg)
        r.sleep()
        
