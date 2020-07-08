#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import time
import unittest

from nose.tools import assert_false
from nose.tools import assert_true

import rospy
import rostopic


PKG = 'jsk_tools'
NAME = 'test_topic_published'


class PublishChecker(object):
    def __init__(self, topic_name, timeout):
        self.topic_name = topic_name
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        msg_class, _, _ = rostopic.get_topic_class(
            rospy.resolve_name(topic_name), blocking=True)
        self.msg = None
        self.sub = rospy.Subscriber(topic_name, msg_class, self._callback)

    def _callback(self, msg):
        self.msg = msg

    def assert_published(self):
        if self.msg:
            return True
        if rospy.Time.now() > self.deadline:
            return False
        return None


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
            if not re.match(r'^topic_\d$', name):
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
        t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn('/use_sim_time is specified and rostime is 0, '
                          '/clock is published?')
            if time.time() - t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            time.sleep(1)

        checkers = []
        for topic_name, timeout, negative in zip(
                self.topics, self.timeouts, self.negatives):
            checker = PublishChecker(topic_name, timeout)
            checkers.append(checker)

        topics_finished = []
        while not rospy.is_shutdown():
            if len(topics_finished) == len(checkers):
                break
            for i, checker in enumerate(checkers):
                if checker.topic_name in topics_finished:
                    continue
                ret = checker.assert_published()
                if ret is None:
                    continue
                topics_finished.append(checker.topic_name)
                if self.negatives[i]:
                    assert_false(ret, 'Topic [%s] is published' %
                                      checker.topic_name)
                else:
                    assert_true(
                        ret, 'Topic [%s] is not published' %
                             checker.topic_name)
            rospy.sleep(0.01)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestTopicPublished, sys.argv)
