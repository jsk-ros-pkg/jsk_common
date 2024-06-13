#!/usr/bin/env python

import sys
import time

import rospy

from std_msgs.msg import String


def main():
    rospy.loginfo("sample1 start")
    pub = rospy.Publisher(
        '/app_scheduler/sample1', String, queue_size=1)
    time.sleep(0.5)
    rospy.loginfo("publishing /app_scheduler/sample1 ...")
    pub.publish(String(data='sample1'))
    rospy.loginfo('sample1 finish')
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('sample1')
    main()
