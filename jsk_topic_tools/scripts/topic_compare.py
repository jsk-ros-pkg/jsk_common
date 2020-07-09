#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
import roslib
import roslib.message
import rospy
import os, sys
import time
import traceback
from threading import Lock

try:
    from jsk_topic_tools.compare import ROSTopicCompare
except ImportError:
    import roslib
    roslib.load_manifest('jsk_topic_tools')
    from jsk_topic_tools.compare import ROSTopicCompare


def onShutdown():
    i = 0
    print("subscribed topic:")
    for sub in tc.subscriberArray:
        print("%d: %s" % (i, sub.name))
        i += 1

def fullUsage():
    print("[Usage] rosrun jsk_topic_tools topic_compare.py [Option] <topic_name_1>..<topic_name_n>")
    print("[Option]")
    print("-h: full usage")
    print("-b: show value as bytes")
    print("-m: show value as megabytes")

if __name__ == '__main__':
    # check if valid argument
    if len(sys.argv) < 2:
        print("[Usage] rosrun jsk_topic_tools topic_compare.py <topic_name_1>..<topic_name_n>")
        quit()

    rospy.init_node("topic_compare")

    # check options
    if "-h" in sys.argv:
        fullUsage()
        quit()
    elif "-b" in sys.argv:
        tc = ROSTopicCompare("B")
    elif "-m" in sys.argv:
        tc = ROSTopicCompare("MB")
    else:
        tc = ROSTopicCompare()

    for name in sys.argv[1:]:
        if name[0] != "/":
            continue
        tc.registerTopic(name)

    rospy.on_shutdown(onShutdown)

    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(1.0)
        tc.printBandWidth()
