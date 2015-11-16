#!/usr/bin/env python
# -*- coding: utf-8 -*-

from mock import patch
import unittest

from jsk_topic_tools.log_utils import jsk_logdebug
from jsk_topic_tools.log_utils import jsk_loginfo
from jsk_topic_tools.log_utils import jsk_logwarn
from jsk_topic_tools.log_utils import jsk_logerr
from jsk_topic_tools.log_utils import jsk_logfatal
from jsk_topic_tools.log_utils import warn_no_remap


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


@patch('rospy.logwarn')
@patch('rospy.names.get_resolved_mappings')
@patch('rosgraph.names.resolve_name')
@patch('rospy.get_name')
def test_warn_no_remap(mock_get_name, mock_resolve_name,
                       mock_get_resolved_mappings, mock_log):
    mock_get_name.return_value = '/spam'
    mock_resolve_name.return_value = '/spam/input'
    mock_get_resolved_mappings.return_value = {}
    warn_no_remap('~input')
    mock_log.assert_called_with("[/spam] '~input' has not been remapped.")
