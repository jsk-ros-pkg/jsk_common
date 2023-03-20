#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import time
import unittest

import rosnode
import rospy
import rostopic

from jsk_topic_tools.eval_utils import expr_eval
from jsk_topic_tools.eval_utils import import_modules


PKG = 'jsk_tools'
NAME = 'test_topic_published'


class PublishChecker(object):
    def __init__(self, topic_name, timeout, condition=None, modules=None):
        modules = modules or {}
        self.topic_name = topic_name
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        msg_class, _, _ = rostopic.get_topic_class(
            rospy.resolve_name(topic_name), blocking=True)
        if condition is not None:
            self.condition_expr = expr_eval(condition, modules)
        self.condition = condition
        self.msg = None
        self.invalid_conditioned_msg = None
        self.sub = rospy.Subscriber(topic_name, msg_class, self._callback)

    def _callback(self, msg):
        if self.condition is not None:
            if self.condition_expr(self.topic_name, msg, rospy.Time.now()):
                self.msg = msg
            else:
                self.invalid_conditioned_msg = msg
        else:
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
        self.conditions = []

        import_list = rospy.get_param('~import', [])
        import_list += ['rospy', 'numpy']
        self.modules = import_modules(import_list)

        params = rospy.get_param(rospy.get_name(), [])
        for name, value in params.items():
            if not re.match(r'^topic_\d$', name):
                continue
            self.topics.append(value)
            id = name.replace('topic_', '')
            self.timeouts.append(rospy.get_param('~timeout_{}'.format(id), 10))
            self.conditions.append(rospy.get_param('~condition_{}'.format(id), None))
            self.negatives.append(
                rospy.get_param('~negative_{}'.format(id), False))
        if not self.topics:
            rospy.logerr('No topic is specified.')
            sys.exit(1)
        self.check_after_kill_node = rospy.get_param(
            '~check_after_kill_node', False)
        if self.check_after_kill_node:
            target_node_names = rospy.get_param('~node_names')
            if not isinstance(target_node_names, list):
                target_node_names = [target_node_names]
            namespace = rospy.get_namespace()
            self.target_node_names = []
            for name in target_node_names:
                if name.startswith('/'):
                    self.target_node_names.append(name)
                else:
                    self.target_node_names.append(namespace + name)

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

        self._check_topic_pubilshed()

        if self.check_after_kill_node:
            rospy.logwarn('Check topic published after killing nodes ({})'.
                          format(self.target_node_names))
            rosnode.kill_nodes(self.target_node_names)
            time.sleep(5.0)
            self._check_topic_pubilshed()

    def _check_topic_pubilshed(self):
        checkers = []
        for topic_name, timeout, negative, condition in zip(
                self.topics, self.timeouts, self.negatives, self.conditions):
            checker = PublishChecker(topic_name, timeout, condition, self.modules)
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
                if self.negatives[i] and ret is True:
                    if checker.condition is None:
                        rospy.logerr('Topic [%s] is published' %
                                     checker.topic_name)
                    else:
                        if checker.msg is not None:
                            rospy.logerr(
                                'Topic [{}] satisfying "{}" is published. '
                                'Topic "{}"'
                                .format(checker.topic_name, checker.condition,
                                        checker.msg))
                        else:
                            rospy.logerr('Topic [%s] is published' %
                                         checker.topic_name)
                    sys.exit(1)
                elif self.negatives[i] is False and ret is False:
                    if checker.condition is None:
                        rospy.logerr('Topic [%s] is not published' %
                                     checker.topic_name)
                    else:
                        if checker.invalid_conditioned_msg is None:
                            rospy.logerr('Topic [%s] is not published' %
                                         checker.topic_name)
                        else:
                            rospy.logerr(
                                'Topic [{}] is published. '
                                'But condition "{}" is not satified. '
                                'Topic "{}"'.format(
                                    checker.topic_name, checker.condition,
                                    checker.invalid_conditioned_msg))
                    sys.exit(1)
            try:
                rospy.sleep(0.01)
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                # Avoid ending test with this sentence
                # if time loops when playing rosbag.
                # See https://github.com/jsk-ros-pkg/jsk_recognition/pull/2682#issuecomment-1120381822
                continue
            except rospy.exceptions.ROSInterruptException:
                not_finished_topics = [topic for topic in self.topics
                                       if topic not in topics_finished]
                rospy.logerr('Not received topic {}'.format(
                    not_finished_topics))
                sys.exit(0)
        if len(topics_finished) != len(checkers):
            raise ValueError('Not all topics are received')


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestTopicPublished, sys.argv)
