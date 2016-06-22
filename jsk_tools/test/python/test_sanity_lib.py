#!/usr/bin/env python

try:
    import cStringIO as StringIO
except ImportError:
    import StringIO
import sys
import unittest

from jsk_tools import sanity_lib
import rospy
import std_msgs.msg


class TestSanityLib(unittest.TestCase):

    def test_TopicPublishedChecker_0(self):
        checker = sanity_lib.TopicPublishedChecker('/input')
        self.assertTrue(checker.check())

    def test_TopicPublishedChecker_1(self):
        sys.stdout = f = StringIO.StringIO()
        checker = sanity_lib.TopicPublishedChecker('/input', echo=True)
        self.assertTrue(checker.check())
        output = f.getvalue()
        sys.stdout = sys.__stdout__
        self.assertIn('/input', output)
        self.assertIn('data: input', output)

    def test_TopicPublishedChecker_2(self):
        checker = sanity_lib.TopicPublishedChecker('/input', data_class=std_msgs.msg.String)
        self.assertTrue(checker.check())


if __name__ == '__main__':
    import rostest
    PKG = 'jsk_tools'
    NAME = 'test_sanity_lib'
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestSanityLib)
