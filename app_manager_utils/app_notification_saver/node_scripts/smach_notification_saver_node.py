#!/usr/bin/env python

import rospy

from app_notification_saver import SmachNotificationSaver

if __name__ == '__main__':
    rospy.init_node('smach_notification_saver_node')
    SmachNotificationSaver()
    rospy.spin()
