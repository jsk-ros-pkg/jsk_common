#!/usr/bin/env python
# -*- coding: utf-8 -*-

import inspect

import rosgraph
import rospy

from jsk_topic_tools.name_utils import unresolve_name


def _log_msg_with_called_location(msg):
    try:
        return '[{cls}::{method}] {msg}'.format(
            cls=inspect.stack()[2][0].f_locals['self'].__class__.__name__,
            method=inspect.stack()[2][0].f_code.co_name,
            msg=msg)
    except KeyError:
        return msg


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
