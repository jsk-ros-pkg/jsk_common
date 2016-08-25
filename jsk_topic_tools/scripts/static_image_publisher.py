#!/usr/bin/env python

import cv2
try:
    from scipy.misc import face
    img = face()[:, :, ::-1]
except ImportError:
    import numpy as np
    from scipy.misc import lena
    img = cv2.cvtColor(lena().astype(np.uint8), cv2.COLOR_GRAY2BGR)

import cv_bridge
import rospy
from sensor_msgs.msg import Image


def timer_cb(event):
    imgmsg.header.stamp = rospy.Time.now()
    pub_img.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('static_image_publisher')

    pub_img = rospy.Publisher('~output', Image, queue_size=1)

    bridge = cv_bridge.CvBridge()
    imgmsg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()
