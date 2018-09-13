#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import sys
import time
import unittest

from nose.tools import assert_false
from nose.tools import assert_true

import rosgraph
import rosnode
import rospy
import rostest


PKG = 'jsk_tools'
NAME = 'test_node_alive'


class AliveChecker(object):
    def __init__(self, node_name, timeout):
        self.node_name = node_name
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        self.master = rosgraph.Master(NAME)

    def assert_alive(self):
        # Try to unregister zombie nodes from ROS master.
        _, unpinged = rosnode.rosnode_ping_all()
        rosnode.cleanup_master_blacklist(self.master, unpinged)

        alive_nodes = rosnode.get_node_names()
        if self.node_name in alive_nodes:
            return True
        if rospy.Time.now() > self.deadline:
            return False
        return None


class TestNodeAlive(unittest.TestCase):
    def __init__(self, *args):
        super(TestNodeAlive, self).__init__(*args)
        rospy.init_node(NAME)
        # nodes to check sanity
        self.nodes = []
        self.timeouts = []
        self.negatives = []
        params = rospy.get_param(rospy.get_name(), [])
        for name, value in params.items():
            if not re.match('^node_\d$', name):
                continue
            self.nodes.append(value)
            id = name.replace('node_', '')
            self.timeouts.append(rospy.get_param('~timeout_{}'.format(id), 10))
            self.negatives.append(
                rospy.get_param('~negative_{}'.format(id), False))
        if not self.nodes:
            rospy.logerr('No node is specified.')
            sys.exit(1)

    def test_node_alive(self):
        """Test if nodes are alive"""
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
        for node_name, timeout, negative in zip(
                self.nodes, self.timeouts, self.negatives):
            checker = AliveChecker(node_name, timeout)
            checkers.append(checker)

        nodes_alive = []
        while not rospy.is_shutdown():
            if len(nodes_alive) == len(checkers):
                break
            for i, checker in enumerate(checkers):
                if checker.node_name in nodes_alive:
                    continue
                ret = checker.assert_alive()
                if ret is None:
                    continue
                nodes_alive.append(checker.node_name)
                if self.negatives[i]:
                    assert_false(
                        ret, 'Node [%s] is alive' % checker.node_name)
                else:
                    assert_true(
                        ret, 'Node [%s] is dead' % checker.node_name)
            rospy.sleep(0.01)


if __name__ == '__main__':
    rostest.run(PKG, NAME, TestNodeAlive, sys.argv)
