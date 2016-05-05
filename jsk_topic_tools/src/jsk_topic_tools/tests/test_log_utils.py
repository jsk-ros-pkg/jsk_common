#!/usr/bin/env python
# -*- coding: utf-8 -*-

from mock import patch
from nose.tools import assert_equal
import unittest

from jsk_topic_tools.log_utils import _log_msg_with_called_location
from jsk_topic_tools.log_utils import jsk_logdebug
from jsk_topic_tools.log_utils import jsk_loginfo
from jsk_topic_tools.log_utils import jsk_logwarn
from jsk_topic_tools.log_utils import jsk_logerr
from jsk_topic_tools.log_utils import jsk_logfatal
from jsk_topic_tools.log_utils import warn_no_remap


@patch('rospy.get_name')
def test__log_msg_with_called_location(mock_get_name):
    mock_get_name.return_value = '/spam'
    msg = _log_msg_with_called_location('spam')
    assert_equal(msg, '[/spam] [patched] spam')


class TestJSKLogXXX(unittest.TestCase):

    @patch('rospy.logdebug')
    @patch('rospy.get_name')
    def test_jsk_logdebug(self, mock_get_name, mock_log):
        mock_get_name.return_value = '/spam'
        jsk_logdebug('spam')
        mock_log.assert_called_with('[/spam] [TestJSKLogXXX::test_jsk_logdebug] spam')

    @patch('rospy.loginfo')
    @patch('rospy.get_name')
    def test_jsk_loginfo(self, mock_get_name, mock_log):
        mock_get_name.return_value = '/spam'
        jsk_loginfo('spam')
        mock_log.assert_called_with('[/spam] [TestJSKLogXXX::test_jsk_loginfo] spam')

    @patch('rospy.logwarn')
    @patch('rospy.get_name')
    def test_jsk_logwarn(self, mock_get_name, mock_log):
        mock_get_name.return_value = '/spam'
        jsk_logwarn('spam')
        mock_log.assert_called_with('[/spam] [TestJSKLogXXX::test_jsk_logwarn] spam')

    @patch('rospy.logerr')
    @patch('rospy.get_name')
    def test_jsk_logerr(self, mock_get_name, mock_log):
        mock_get_name.return_value = '/spam'
        jsk_logerr('spam')
        mock_log.assert_called_with('[/spam] [TestJSKLogXXX::test_jsk_logerr] spam')

    @patch('rospy.logfatal')
    @patch('rospy.get_name')
    def test_jsk_logfatal(self, mock_get_name, mock_log):
        mock_get_name.return_value = '/spam'
        jsk_logfatal('spam')
        mock_log.assert_called_with('[/spam] [TestJSKLogXXX::test_jsk_logfatal] spam')


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
