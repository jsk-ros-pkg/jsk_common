#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest

import numpy as np

import rospy
try:
    from std_msgs.msg import Float32
except:
    import roslib
    roslib.load_manifest("jsk_topic_tools")
    from std_msgs.msg import Float32


PKG = 'jsk_topic_tools'
NAME = 'test_hz_measure'


def eps_equal(a, b, err=0.001):
    return abs(a - b) < err


class TestHzMeasure(unittest.TestCase):

    def __init__(self, *args):
        super(TestHzMeasure, self).__init__(*args)
        self.hz_msg = None
        rospy.init_node(NAME)
        rospy.Subscriber('/hz/output', Float32, self.topic_cb)

    def topic_cb(self, msg):
        self.hz_msg = msg

    def test_hz(self):
        while self.hz_msg is None:
            rospy.loginfo('wait for msg...')
            if not rospy.is_shutdown():
                rospy.sleep(5.0)  # wait for 5 sec
        # should be 30Hz
        msgs = []
        while len(msgs) < 30:
            msgs.append(self.hz_msg.data)
            # rospy.loginfo('hz of msg %s'%hz_msg.data)
            rospy.sleep(0.1)
        hz = np.median(msgs)
        rospy.loginfo('average hz of msg %s' % np.mean(msgs))
        rospy.loginfo('median  hz of msg %s' % hz)
        self.assertTrue(eps_equal(hz, 30, 1))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestHzMeasure)
