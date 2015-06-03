#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import FC2OCS

if __name__ == "__main__":
    rospy.init_node("fc2ocs_test_publisher")
    pub = rospy.Publisher("fc2ocs_original", FC2OCS)
    pub_rate = rospy.get_param("~pub_rate", 1000)
    r = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        msg = FC2OCS()
        msg.joint_angles = [10] * 32
        msg.servo_state = True
        pub.publish(msg)
        r.sleep()
        
