#!/usr/bin/env python

import rospy

from app_notification_saver import ServiceNotificationSaver


if __name__ == '__main__':
    rospy.init_node('service_notification_saver_node')
    ServiceNotificationSaver()
    rospy.spin()
