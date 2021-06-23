#!/usr/bin/env python

from __future__ import print_function

import unittest
import os

from jsk_topic_tools.master_util import isMasterAlive
import rospy


PKG = 'jsk_topic_tools'
NAME = 'test_python_master_util'


class TestPythonMasterUtil(unittest.TestCase):
    def setUp(self):
        self.original_uri = os.environ["ROS_MASTER_URI"]

    def tearDown(self):
        os.environ["ROS_MASTER_URI"] = self.original_uri

    def test_isMasterAlive(self):
        self.assertTrue(isMasterAlive())
        self.assertTrue(isMasterAlive(10,1))
        os.environ["ROS_MASTER_URI"] = 'http://baduri:11311'
        self.assertFalse(isMasterAlive())
        self.assertFalse(isMasterAlive(10,1))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestPythonMasterUtil)
