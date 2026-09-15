#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS1 (rospy) / ROS2 (rclpy) compatibility helpers, shared across
jsk_common's ROS2-ported packages (jsk_tools, jsk_data,
dynamic_tf_publisher, ...). See jsk_rqt_plugins/src/jsk_rqt_plugins/
ros_compat.py (jsk_visualization) for the original version of this
pattern, which this is adapted from.

Nodes using this module take an optional ``node`` argument: ``None``
under ROS1 (the classic rospy singleton API is used directly), or the
owning ``rclpy.node.Node`` under ROS2.

Packages depend on jsk_ros1_ros2_compat (a build/exec_depend) and
import this as ``from jsk_ros1_ros2_compat.rospy_rclpy_compat import
...``. jsk_ros1_ros2_compat has no dependencies of its own on any other
jsk_common package, so depending on it never risks a dependency cycle.
"""
import os
import time

import tf2_ros

ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))

if ROS_VERSION == 2:
    import rclpy  # NOQA: F401 (re-exported for convenience)
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.duration import Duration as _Duration
    from rosidl_runtime_py.utilities import get_message
else:
    import roslib.message
    import rospy


def _declare_dynamic(node, name, default):
    # dynamic_typing=True avoids a rigid declared-type that would
    # otherwise conflict with a differently-typed value provided via
    # a launch parameters-file (e.g. int default vs float override).
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))


def private_name(name):
    """ROS1 accepts a bare '~foo'; ROS2 requires the slash: '~/foo'."""
    if ROS_VERSION == 2 and name.startswith('~') and not name.startswith('~/'):
        return '~/' + name[1:]
    return name


def get_param(node, name, default=None):
    if ROS_VERSION == 2:
        name = name[1:] if name.startswith('~') else name
        if not node.has_parameter(name):
            _declare_dynamic(node, name, default)
        value = node.get_parameter(name).value
        return default if value is None else value
    else:
        return rospy.get_param(name, default)


def has_param(node, name):
    if ROS_VERSION == 2:
        name = name[1:] if name.startswith('~') else name
        return node.has_parameter(name) and node.get_parameter(name).value is not None
    else:
        return rospy.has_param(name)


def loginfo(node, msg):
    node.get_logger().info(msg) if ROS_VERSION == 2 else rospy.loginfo(msg)


def logwarn(node, msg):
    node.get_logger().warn(msg) if ROS_VERSION == 2 else rospy.logwarn(msg)


def logerr(node, msg):
    node.get_logger().error(msg) if ROS_VERSION == 2 else rospy.logerr(msg)


def logdebug(node, msg):
    node.get_logger().debug(msg) if ROS_VERSION == 2 else rospy.logdebug(msg)


def sleep(node, seconds):
    time.sleep(seconds) if ROS_VERSION == 2 else rospy.sleep(seconds)


def is_shutdown(node):
    return (not rclpy.ok()) if ROS_VERSION == 2 else rospy.is_shutdown()


class _RateShim(object):
    """ROS2 has no rospy.Rate equivalent that sleeps without an executor
    spinning it; a plain period-based time.sleep() is an adequate
    stand-in for the coarse polling loops this is used for."""

    def __init__(self, hz):
        self._period = 1.0 / hz

    def sleep(self):
        time.sleep(self._period)


def Rate(node, hz):
    return rospy.Rate(hz) if ROS_VERSION == 1 else _RateShim(hz)


def now_sec(node):
    if ROS_VERSION == 2:
        return node.get_clock().now().nanoseconds * 1e-9
    else:
        return rospy.Time.now().to_sec()


def stamp_to_sec(stamp):
    """stamp: a message-carried time field (e.g. Header.stamp)."""
    if ROS_VERSION == 2:
        return stamp.sec + stamp.nanosec * 1e-9
    else:
        return stamp.to_sec()


def stamp_now(node):
    """A Header.stamp-compatible "now" value."""
    return node.get_clock().now().to_msg() if ROS_VERSION == 2 else rospy.Time.now()


def resolve_message_class(type_str):
    """Accepts either ROS1-style ("pkg/Type") or ROS2-style
    ("pkg/msg/Type") message type strings."""
    if ROS_VERSION == 2:
        try:
            return get_message(type_str)
        except (ValueError, AttributeError, ModuleNotFoundError):
            pkg, _, name = type_str.partition('/')
            return get_message('{}/msg/{}'.format(pkg, name))
    else:
        pkg, _, name = type_str.rpartition('/')
        return roslib.message.get_message_class('{}/{}'.format(pkg, name))


def create_subscription(node, topic, msg_type, callback, queue_size=1):
    if ROS_VERSION == 2:
        return node.create_subscription(msg_type, topic, callback, queue_size)
    else:
        return rospy.Subscriber(topic, msg_type, callback)


def create_publisher(node, topic, msg_type, queue_size=1):
    if ROS_VERSION == 2:
        return node.create_publisher(msg_type, private_name(topic), queue_size)
    else:
        return rospy.Publisher(topic, msg_type, queue_size=queue_size)


def create_timer(node, period_sec, callback):
    if ROS_VERSION == 2:
        return node.create_timer(period_sec, lambda: callback(None))
    else:
        return rospy.Timer(rospy.Duration(period_sec), callback)


def create_service(node, name, srv_type, callback):
    """Generic form: `callback` takes (request, response) and returns
    the (possibly same) response, for both ROS1 and ROS2."""
    if ROS_VERSION == 2:
        return node.create_service(srv_type, name, callback)
    else:
        def _wrapped(request):
            response = srv_type._response_class()
            return callback(request, response)
        return rospy.Service(name, srv_type, _wrapped)


def create_trigger_service(node, service_name, service_type, callback):
    """Trigger-shaped form: `callback` takes a single ROS1-style request
    argument and returns (success: bool, message: str)."""
    def _wrapped(request, response):
        success, message = callback(request)
        response.success = success
        response.message = message
        return response
    return create_service(node, private_name(service_name), service_type, _wrapped)


def call_service(node, name, srv_type, request, timeout=5.0):
    """Call the service at `name` with `request` and return the response."""
    if ROS_VERSION == 2:
        client = node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('%s service is not available' % (name,))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    else:
        rospy.wait_for_service(name, timeout)
        client = rospy.ServiceProxy(name, srv_type)
        return client(request)


def create_request(srv_type):
    return srv_type.Request() if ROS_VERSION == 2 else srv_type._request_class()


def create_response(srv_type):
    return srv_type.Response() if ROS_VERSION == 2 else srv_type._response_class()


def duration_from_sec(seconds):
    return _Duration(seconds=seconds) if ROS_VERSION == 2 else rospy.Duration(seconds)


def create_tf_buffer_and_listener(node):
    """tf2_ros.Buffer()+TransformListener(buffer) (ROS1) vs
    TransformListener(buffer, node) (ROS2, needs a node for its
    subscription/timer callbacks)."""
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node) \
        if ROS_VERSION == 2 else tf2_ros.TransformListener(buffer)
    return buffer, listener


def create_tf_broadcaster(node):
    return tf2_ros.TransformBroadcaster(node) if ROS_VERSION == 2 else tf2_ros.TransformBroadcaster()


def declare_params(node, defaults):
    """ROS2 only: node.declare_parameter() for every name/default pair
    in the `defaults` dict, in one call."""
    for name, default in defaults.items():
        node.declare_parameter(name, default)


def get_params(node, names):
    """ROS2 only: {name: node.get_parameter(name).value for name in names}."""
    return {name: node.get_parameter(name).value for name in names}


def spin_and_shutdown(node, executor=None):
    """ROS2 only: spin `node` (via `executor` if given, else a plain
    rclpy.spin(node)) until Ctrl-C, then destroy_node()/shutdown().

    Assumes rclpy.init() was already called and `node` already
    constructed; this covers only the boilerplate that is otherwise
    duplicated verbatim across every ROS2-ported node's main()."""
    try:
        if executor is not None:
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
