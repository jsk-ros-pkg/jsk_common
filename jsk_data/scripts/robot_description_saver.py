#!/usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node("robot_description_saver")

robot_description = rospy.get_param("robot_description")
rate = rospy.get_param("~rate", 0.1)      #0.1Hz
pub = rospy.Publisher("robot_description", String)

def timerCallback(event):
    global pub, robot_description
    pub.publish(String(data=robot_description))

timer = rospy.Timer(rospy.Duration(1.0 / rate), timerCallback)
rospy.spin()

    
