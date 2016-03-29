#!/usr/bin/env python

import os.path as p
import cv2
from cv_bridge import CvBridge, CvBridgeError

import unittest

import rosnode
import rosgraph
from sensor_msgs.msg import Image

import rospy


PKG = 'jsk_data'
ID = 'test_rosbag_for_rviz'


class TestRosbagForRviz(unittest.TestCase):

    def setUp(self):
        self.master = rosgraph.Master(ID)

    def test_rosbag_for_rviz(self):
        node_name = 'rosbag_for_rviz'
        for _ in xrange(10):
            state = self.master.getSystemState()
            subs = [t for t, l in state[1] if '/' + node_name in l]
            if subs:
                break
            rospy.logwarn('Waiting for rosbag_for_rviz in 1 sec..')
            rospy.sleep(1)
        self.assertListEqual(subs, ['/dummy_0'])


if __name__ == "__main__":
    import rostest
    rospy.init_node(ID)
    rostest.rosrun(PKG, ID, TestRosbagForRviz)
