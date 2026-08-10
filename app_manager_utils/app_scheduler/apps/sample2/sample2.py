#!/usr/bin/env python

import sys
import time

import rospy

from std_msgs.msg import String


def main():
    rospy.loginfo("sample2 start")
    pub = rospy.Publisher(
        '/app_scheduler/sample2', String, queue_size=1)
    time.sleep(0.5)
    rospy.loginfo("publishing /app_scheduler/sample2 ...")
    pub.publish(String(data='sample2'))
    rospy.loginfo('sample2 finish')
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('sample2')
    main()
