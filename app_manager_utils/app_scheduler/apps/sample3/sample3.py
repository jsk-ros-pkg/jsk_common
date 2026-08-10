#!/usr/bin/env python

import sys
import time

import rospy

from std_msgs.msg import String


def main():
    rospy.loginfo("sample3 start")
    input_value = rospy.get_param('~input', 'default')
    rospy.loginfo('input: {}'.format(input_value))
    pub = rospy.Publisher(
        '/app_scheduler/sample3', String, queue_size=1)
    time.sleep(0.5)
    rospy.loginfo("publishing /app_scheduler/sample3 ...")
    pub.publish(String(data=input_value))
    rospy.loginfo('sample3 finish')
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('sample3')
    main()
