#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from scipy.misc import lena

import rospy
from sensor_msgs.msg import Image
import cv_bridge


def main():
    pub = rospy.Publisher('image', Image, queue_size=1)
    img = cv2.cvtColor(lena().astype(np.uint8), cv2.COLOR_GRAY2BGR)
    bridge = cv_bridge.CvBridge()
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    msg.header.frame_id = 'camera'
    # publish in 30 hz
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.get_rostime()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('publish_lena')
    main()
