#!/usr/bin/env python
import roslib; roslib.load_manifest('pddl_msgs')
import rospy
from pddl_msgs.msg import *

def talker():
    pub = rospy.Publisher('chatter', PDDLhoge)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = ["hello world %s"%rospy.get_time(), "aaa", "bbb"]
        rospy.loginfo(str)
        pub.publish(a = str)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
    
