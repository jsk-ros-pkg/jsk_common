#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import FC2OSC

if __name__ == "__main__":
    rospy.init_node("fc2osc_test_publisher")
    pub = rospy.Publisher("fc2osc_original", FC2OSC)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = FC2OSC()
        msg.joint_angles = [10] * 32
        msg.servo_state = True
        pub.publish(msg)
        r.sleep()
        
