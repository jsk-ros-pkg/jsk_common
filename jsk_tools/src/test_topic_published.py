#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import unittest

from nose.tools import assert_true
from nose.tools import assert_false

import rospy
from jsk_tools.sanity_lib import TopicPublishedChecker


PKG = 'jsk_tools'
NAME = 'test_topic_published'


class TestTopicPublished(unittest.TestCase):
    def __init__(self, *args):
        super(TestTopicPublished, self).__init__(*args)
        rospy.init_node(NAME)
        # topics to check sanity
        self.topics = []
        self.timeouts = []
        self.negatives = []
        params = rospy.get_param(rospy.get_name(), [])
        for name, value in params.items():
            if not re.match('^topic_\d$', name):
                continue
            self.topics.append(value)
            id = name.replace('topic_', '')
            self.timeouts.append(rospy.get_param('~timeout_{}'.format(id)))
            self.negatives.append(
                rospy.get_param('~negative_{}'.format(id), False))
        if not self.topics:
            rospy.logerr('No topic is specified.')
            sys.exit(1)

    def test_published(self):
        """Test topics are published and messages come"""
        checkers = []
        for topic_name, timeout in zip(self.topics, self.timeouts):
            checker = TopicPublishedChecker(topic_name, timeout)
            checkers.append(checker)
        for negative, checker in zip(self.negatives, checkers):
            if negative:
                assert_false(checker.check())
            else:
                assert_true(checker.check())


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestTopicPublished, sys.argv)
