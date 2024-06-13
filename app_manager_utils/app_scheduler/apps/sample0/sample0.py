#!/usr/bin/env python

import sys
import time

import rospy

from std_msgs.msg import String


def main():
    rospy.loginfo("sample0 start")
    pub = rospy.Publisher(
        '/app_scheduler/sample0', String, queue_size=1)
    time.sleep(0.5)
    rospy.loginfo("publishing /app_scheduler/sample0 ...")
    pub.publish(String(data='sample0'))
    rospy.loginfo('sample0 finish')
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('sample0')
    main()
