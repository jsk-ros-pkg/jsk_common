#!/usr/bin/env python
# -*- coding: utf-8 -*-

from mock import patch
import unittest
from nose.tools import assert_equal

import rospy

from jsk_topic_tools.log_utils import jsk_logdebug
from jsk_topic_tools.log_utils import jsk_loginfo
from jsk_topic_tools.log_utils import jsk_logwarn
from jsk_topic_tools.log_utils import jsk_logerr
from jsk_topic_tools.log_utils import jsk_logfatal


class TestJSKLogXXX(unittest.TestCase):

    @patch('rospy.logdebug')
    def test_jsk_logdebug(self, mock_log):
        jsk_logdebug('spam')
        mock_log.assert_called_with('[TestJSKLogXXX::test_jsk_logdebug] spam')

    @patch('rospy.loginfo')
    def test_jsk_loginfo(self, mock_log):
        jsk_loginfo('spam')
        mock_log.assert_called_with('[TestJSKLogXXX::test_jsk_loginfo] spam')

    @patch('rospy.logwarn')
    def test_jsk_logwarn(self, mock_log):
        jsk_logwarn('spam')
        mock_log.assert_called_with('[TestJSKLogXXX::test_jsk_logwarn] spam')

    @patch('rospy.logerr')
    def test_jsk_logerr(self, mock_log):
        jsk_logerr('spam')
        mock_log.assert_called_with('[TestJSKLogXXX::test_jsk_logerr] spam')

    @patch('rospy.logfatal')
    def test_jsk_logfatal(self, mock_log):
        jsk_logfatal('spam')
        mock_log.assert_called_with('[TestJSKLogXXX::test_jsk_logfatal] spam')
