#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import FC2OCS

if __name__ == "__main__":
    rospy.init_node("fc2ocs_test_publisher")
    pub = rospy.Publisher("fc2ocs_original", FC2OCS)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = FC2OCS()
        msg.joint_angles = [10] * 32
        msg.servo_state = True
        pub.publish(msg)
        r.sleep()
        
