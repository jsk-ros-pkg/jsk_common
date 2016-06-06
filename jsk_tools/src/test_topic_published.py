#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import unittest

from nose.tools import assert_true
from nose.tools import assert_false

import rospy
import rostopic


PKG = 'jsk_tools'
NAME = 'test_topic_published'


class TopicPublishedChecker(object):

    """Utility clas to topic is published"""

    def __init__(self, topic_name, timeout=10):
        self.topic_name = topic_name
        self.msg = None
        rospy.loginfo('Getting message class for topic [%s]' % topic_name)
        msg_class = rostopic.get_topic_class(topic_name, blocking=True)[0]
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        self.sub = rospy.Subscriber(topic_name, msg_class, self._callback)

    def check(self):
        while not rospy.is_shutdown():
            if (self.msg is None) and (rospy.Time.now() < self.deadline):
                rospy.sleep(0.1)
            else:
                break
        return self.msg is not None

    def _callback(self, msg):
        if rospy.Time.now() < self.deadline:
            self.msg = msg


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
            self.timeouts.append(rospy.get_param('~timeout_{}'.format(id), 10))
            self.negatives.append(
                rospy.get_param('~negative_{}'.format(id), False))
        if not self.topics:
            rospy.logerr('No topic is specified.')
            sys.exit(1)

    def test_published(self):
        """Test topics are published and messages come"""
        use_sim_time = rospy.get_param('/use_sim_time', False)
        while use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn('/use_sim_time is specified and rostime is 0, /clock is published?')
            rospy.sleep(0.1)
        checkers = []
        for topic_name, timeout in zip(self.topics, self.timeouts):
            checker = TopicPublishedChecker(topic_name, timeout)
            checkers.append(checker)
        for negative, checker in zip(self.negatives, checkers):
            if negative:
                msg = '{} is published'.format(checker.topic_name)
                assert_false(checker.check(), msg)
            else:
                msg = '{} is not published'.format(checker.topic_name)
                assert_true(checker.check(), msg)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestTopicPublished, sys.argv)
