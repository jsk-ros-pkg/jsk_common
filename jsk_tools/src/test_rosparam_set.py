#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import time
import unittest

from nose.tools import assert_true

import rospy


PKG = 'jsk_tools'
NAME = 'test_rosparam_set'


class ROSParamChecker(object):
    def __init__(self, param_name, expected, timeout):
        self.param_name = param_name
        self.expected = expected
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)

    def assert_rosparam(self):
        param = rospy.get_param(self.param_name, None)
        if param == self.expected:
            return True
        if rospy.Time.now() > self.deadline:
            return False
        return None


class TestROSParamSet(unittest.TestCase):
    def __init__(self, *args):
        super(TestROSParamSet, self).__init__(*args)
        rospy.init_node(NAME)
        # params to check sanity
        self.param_names = []
        self.timeouts = []
        self.expecteds = []
        params = rospy.get_param('~params', None)
        for param in params:
            self.param_names.append(param['name'])
            self.expecteds.append(param['expected'])
            self.timeouts.append(param['timeout'])
        if not self.param_names:
            rospy.logerr('No ROS param is specified.')
            sys.exit(1)

    def test_set(self):
        """Test ROS params are correctly set"""
        use_sim_time = rospy.get_param('/use_sim_time', False)
        t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn('/use_sim_time is specified and rostime is 0, '
                          '/clock is published?')
            if time.time() - t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            time.sleep(1)

        checkers = []
        for param_name, timeout, expected in zip(
                self.param_names, self.timeouts, self.expecteds):
            checker = ROSParamChecker(param_name, expected, timeout)
            checkers.append(checker)

        params_finished = []
        while not rospy.is_shutdown():
            if len(params_finished) == len(checkers):
                break
            for i, checker in enumerate(checkers):
                if checker.param_name in params_finished:
                    continue
                ret = checker.assert_rosparam()
                if ret is None:
                    continue
                params_finished.append(checker.param_name)
                assert_true(
                    ret, 'ROS param [%s] is not correctly set' %
                         checker.param_name)
            rospy.sleep(0.01)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestROSParamSet, sys.argv)
