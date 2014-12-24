#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime
from jsk_topic_tools.master_util import isMasterAlive

def check_master():
    return isMasterAlive()

if __name__ == '__main__':
    rospy.init_node('roscore_check')

    while check_master() and not rospy.is_shutdown():
        time_pub = rospy.Publisher("/time", String)
        print "----------------------"
        d = datetime.now()
        print d
        msg = String()
        msg.data = str(d)
        time_pub.publish(msg)
        rospy.sleep(1)
