#!/usr/bin/env python
# -*- coding: utf-8 -*-

# cPickle is moved to _pickle in Python3
# See https://docs.python.org/3.1/whatsnew/3.0.html#library-changes
try:
    import cPickle as pickle
except ImportError:
    import _pickle as pickle
import inspect
import os

ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))

if ROS_VERSION == 2:
    # ROS2 has no free-standing logging macros or global node name --
    # RCLCPP-style logger calls always need a node, so callers must
    # register theirs once via set_node() before using jsk_log*() below.
    g_node = None

    def set_node(node):
        global g_node
        g_node = node

    def _node_name():
        return g_node.get_name()

    def _now_sec():
        return g_node.get_clock().now().nanoseconds * 1e-9

    def logdebug(msg):
        g_node.get_logger().debug(msg)

    def loginfo(msg):
        g_node.get_logger().info(msg)

    def logwarn(msg):
        g_node.get_logger().warn(msg)

    def logerr(msg):
        g_node.get_logger().error(msg)

    def logfatal(msg):
        g_node.get_logger().fatal(msg)
else:
    import rospy

    def set_node(node):
        # No-op under ROS1: rospy's logging/name APIs are free-standing
        # (no node object needed), unlike ROS2's RCLCPP-style logger
        # calls. Exists so callers don't need their own ROS_VERSION
        # branch just to call set_node().
        pass

    def _node_name():
        return rospy.get_name()

    def _now_sec():
        return rospy.Time.now().to_sec()

    # Not `logdebug = rospy.logdebug`-style aliasing: that binds to
    # rospy.logdebug's value once, at import time, which would not see
    # a later `@mock.patch('rospy.logdebug')` (tests patch the
    # attribute on the rospy module, not this module's own binding).
    def logdebug(msg):
        rospy.logdebug(msg)

    def loginfo(msg):
        rospy.loginfo(msg)

    def logwarn(msg):
        rospy.logwarn(msg)

    def logerr(msg):
        rospy.logerr(msg)

    def logfatal(msg):
        rospy.logfatal(msg)


def _log_msg_with_called_location(msg):
    try:
        return '[{node}] [{cls}::{method}] {msg}'.format(
            node=_node_name(),
            cls=inspect.stack()[2][0].f_locals['self'].__class__.__name__,
            method=inspect.stack()[2][0].f_code.co_name,
            msg=msg)
    except KeyError:
        return '[{node}] [{func}] {msg}'.format(
            node=_node_name(),
            func=inspect.stack()[2][0].f_code.co_name,
            msg=msg)


def jsk_logdebug(msg):
    logdebug(_log_msg_with_called_location(msg))


def jsk_loginfo(msg):
    loginfo(_log_msg_with_called_location(msg))


def jsk_logwarn(msg):
    logwarn(_log_msg_with_called_location(msg))


def jsk_logerr(msg):
    logerr(_log_msg_with_called_location(msg))


def jsk_logfatal(msg):
    logfatal(_log_msg_with_called_location(msg))


class LoggingThrottle(object):

    last_logging_time_table = {}

    def __call__(self, id, logging_func, period, msg):
        """Do logging specified message periodically.

        - id (str): Id to identify the caller
        - logging_func (function): Function to do logging.
        - period (float): Period to do logging in second unit.
        - msg (object): Message to do logging.
        """
        now = _now_sec()

        last_logging_time = self.last_logging_time_table.get(id)

        if (last_logging_time is None or
              (now - last_logging_time) > period):
            logging_func(msg)
            self.last_logging_time_table[id] = now


_logging_throttle = LoggingThrottle()


def logdebug_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, logdebug, period, msg)


def loginfo_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, loginfo, period, msg)


def logwarn_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, logwarn, period, msg)


def logerr_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, logerr, period, msg)


def logfatal_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, logfatal, period, msg)
