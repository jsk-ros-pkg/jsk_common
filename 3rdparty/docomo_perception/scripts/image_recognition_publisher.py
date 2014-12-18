#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.srv import ImageRecognition, ImageRecognitionRequest
from sensor_msgs.msg import Image

import sys

global callbackReceived

def cb(msg):
    global callbackReceived

    res = image_recognition(type=ImageRecognitionRequest.ALL,
                            image=msg,
                            numOfCandidates=10)
    for c in res.candidates:
        rospy.loginfo("- - - - - -")
        rospy.loginfo("itemname: %s", c.itemName)
        rospy.loginfo("category: %s", c.category)
        rospy.loginfo("score: %f", c.score)
        rospy.loginfo("imageUrl: %s", c.imageUrl)

    callbackReceived = True

if __name__ == '__main__':
    rospy.init_node("image_recognition_publisher")

    callbackReceived = False

    rospy.wait_for_service("image_recognition")
    image_recognition = rospy.ServiceProxy("image_recognition", ImageRecognition)

    sub = rospy.Subscriber("image", Image, cb)

    while not rospy.is_shutdown() and not callbackReceived:
        rospy.sleep(1)
