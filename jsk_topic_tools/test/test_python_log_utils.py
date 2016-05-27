#!/usr/bin/env python

from __future__ import print_function

import cStringIO as StringIO
import sys
import unittest

from jsk_topic_tools.log_utils import LoggingThrottle
import rospy


PKG = 'jsk_topic_tools'
NAME = 'test_python_log_utils'


class TestPythonLogUtils(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)
        self.logging_throttle = LoggingThrottle()

    def _check(self, no_logging=False):
        sys.stdout = f = StringIO.StringIO()
        self.logging_throttle(id='a', logging_func=print, period=3, msg='spam')
        sys.stdout = sys.__stdout__
        if no_logging:
            self.assertFalse(f.getvalue(), 'spam\n')
        else:
            self.assertEqual(f.getvalue(), 'spam\n')

    def test_LoggingThrottle(self):
        self._check()
        self._check(no_logging=True)
        rospy.sleep(rospy.Duration(3))
        self._check()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestPythonLogUtils)
