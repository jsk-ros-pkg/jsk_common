#!/usr/bin/env python

import rospy

from std_msgs.msg import String
rospy.init_node("robot_description_saver")

def callback(msg):
    rospy.set_param("robot_description", msg.data)

sub = rospy.Subscriber("robot_description", String, callback)

rospy.spin()
