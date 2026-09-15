#!/usr/bin/env python
# -*- coding: utf-8 -*-

# jsk_logdebug/jsk_loginfo/jsk_logwarn/jsk_logerr/jsk_logfatal, their
# *_throttle variants, and set_node() (ROS2 only) live in
# jsk_ros1_ros2_compat -- shared with other packages rather than
# duplicated here. warn_no_remap() stays below: it needs
# jsk_topic_tools.name_utils, and is ROS1-only anyway (ROS2 has no
# rosgraph.names.resolve_name/get_resolved_mappings equivalent).
from jsk_ros1_ros2_compat.log_utils import ROS_VERSION
from jsk_ros1_ros2_compat.log_utils import set_node  # NOQA: F401 (ROS2 only, re-exported)
from jsk_ros1_ros2_compat.log_utils import jsk_logdebug  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import jsk_loginfo  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import jsk_logwarn  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import jsk_logerr  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import jsk_logfatal  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import logdebug_throttle  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import loginfo_throttle  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import logwarn_throttle  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import logerr_throttle  # NOQA: F401
from jsk_ros1_ros2_compat.log_utils import logfatal_throttle  # NOQA: F401

from jsk_topic_tools.name_utils import unresolve_name

if ROS_VERSION == 1:
    import rosgraph
    import rospy


def warn_no_remap(*names):
    # ROS2 has no equivalent remapping-introspection API
    # (rosgraph.names.resolve_name / get_resolved_mappings), so this
    # stays ROS1-only.
    if ROS_VERSION == 2:
        return
    node_name = rospy.get_name()
    resolved_names = [rosgraph.names.resolve_name(n, node_name) for n in names]
    mappings = rospy.names.get_resolved_mappings()
    for r_name in resolved_names:
        if r_name in mappings:
            continue
        name = unresolve_name(node_name, r_name)
        rospy.logwarn("[{node_name}] '{name}' has not been remapped."
                      .format(node_name=node_name, name=name))
