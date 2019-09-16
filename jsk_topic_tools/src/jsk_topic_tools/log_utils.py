#!/usr/bin/env python
# -*- coding: utf-8 -*-

# cPickle is moved to _pickle in Python3
# See https://docs.python.org/3.1/whatsnew/3.0.html#library-changes
try:
    import cPickle as pickle
except ImportError:
    import _pickle as pickle
import inspect

import rosgraph
import rospy

from jsk_topic_tools.name_utils import unresolve_name


def _log_msg_with_called_location(msg):
    try:
        return '[{node}] [{cls}::{method}] {msg}'.format(
            node=rospy.get_name(),
            cls=inspect.stack()[2][0].f_locals['self'].__class__.__name__,
            method=inspect.stack()[2][0].f_code.co_name,
            msg=msg)
    except KeyError:
        return '[{node}] [{func}] {msg}'.format(
            node=rospy.get_name(),
            func=inspect.stack()[2][0].f_code.co_name,
            msg=msg)


def jsk_logdebug(msg):
    rospy.logdebug(_log_msg_with_called_location(msg))


def jsk_loginfo(msg):
    rospy.loginfo(_log_msg_with_called_location(msg))


def jsk_logwarn(msg):
    rospy.logwarn(_log_msg_with_called_location(msg))


def jsk_logerr(msg):
    rospy.logerr(_log_msg_with_called_location(msg))


def jsk_logfatal(msg):
    rospy.logfatal(_log_msg_with_called_location(msg))


class LoggingThrottle(object):

    last_logging_time_table = {}

    def __call__(self, id, logging_func, period, msg):
        """Do logging specified message periodically.

        - id (str): Id to identify the caller
        - logging_func (function): Function to do logging.
        - period (float): Period to do logging in second unit.
        - msg (object): Message to do logging.
        """
        now = rospy.Time.now()

        last_logging_time = self.last_logging_time_table.get(id)

        if (last_logging_time is None or
              (now - last_logging_time) > rospy.Duration(period)):
            logging_func(msg)
            self.last_logging_time_table[id] = now


_logging_throttle = LoggingThrottle()


def logdebug_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, rospy.logdebug, period, msg)


def loginfo_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, rospy.loginfo, period, msg)


def logwarn_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, rospy.logwarn, period, msg)


def logerr_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, rospy.logerr, period, msg)


def logfatal_throttle(period, msg):
    id = pickle.dumps(inspect.stack()[1][1:])
    _logging_throttle(id, rospy.logfatal, period, msg)


def warn_no_remap(*names):
    node_name = rospy.get_name()
    resolved_names = [rosgraph.names.resolve_name(n, node_name) for n in names]
    mappings = rospy.names.get_resolved_mappings()
    for r_name in resolved_names:
        if r_name in mappings:
            continue
        name = unresolve_name(node_name, r_name)
        rospy.logwarn("[{node_name}] '{name}' has not been remapped."
                      .format(node_name=node_name, name=name))
