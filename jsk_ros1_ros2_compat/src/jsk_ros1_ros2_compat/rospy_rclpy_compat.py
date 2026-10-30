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
    from rclpy.executors import ExternalShutdownException
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


def logfatal(node, msg):
    node.get_logger().fatal(msg) if ROS_VERSION == 2 else rospy.logfatal(msg)


def loginfo_throttle(node, period, msg):
    if ROS_VERSION == 2:
        node.get_logger().info(msg, throttle_duration_sec=period)
    else:
        rospy.loginfo_throttle(period, msg)


def logwarn_throttle(node, period, msg):
    if ROS_VERSION == 2:
        node.get_logger().warn(msg, throttle_duration_sec=period)
    else:
        rospy.logwarn_throttle(period, msg)


def logerr_throttle(node, period, msg):
    if ROS_VERSION == 2:
        node.get_logger().error(msg, throttle_duration_sec=period)
    else:
        rospy.logerr_throttle(period, msg)


def get_name(node):
    """Fully qualified node name, e.g. '/ns/my_node' (rospy.get_name())."""
    return node.get_fully_qualified_name() if ROS_VERSION == 2 else rospy.get_name()


def signal_shutdown(node, reason):
    """rospy.signal_shutdown() (ROS1) vs shutting down the rclpy context
    (ROS2), which makes a spin() in progress (e.g. spin_and_shutdown())
    return."""
    if ROS_VERSION == 2:
        node.get_logger().info('shutdown request: %s' % (reason,))
        rclpy.try_shutdown()
    else:
        rospy.signal_shutdown(reason)


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


def create_subscription(node, topic, msg_type, callback, queue_size=None):
    """queue_size=None keeps each side's default: an unbounded queue
    (rospy) or a depth-1 QoS history (rclpy, which requires a depth)."""
    if ROS_VERSION == 2:
        return node.create_subscription(
            msg_type, topic, callback, 1 if queue_size is None else queue_size)
    else:
        return rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)


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
    duplicated verbatim across every ROS2-ported node's main().

    rclpy's default signal handlers turn SIGINT/SIGTERM into an
    ExternalShutdownException out of spin() (a KeyboardInterrupt only if
    it arrives outside the executor), so both mean a normal exit."""
    try:
        if executor is not None:
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def on_shutdown(node, callback):
    """rospy.on_shutdown() (ROS1) vs the rclpy context's shutdown hook
    (ROS2); `callback` takes no argument in both."""
    if ROS_VERSION == 2:
        node.context.on_shutdown(callback)
    else:
        rospy.on_shutdown(callback)


class _SimpleActionClient(object):
    """actionlib.SimpleActionClient-like wrapper of rclpy's ActionClient,
    for the commonly used subset. Unlike rclpy's own API, the blocking
    calls work from any thread other than the one spinning `node`
    (e.g. a MultiThreadedExecutor callback or a worker thread), and
    never from a single-threaded executor's own callback."""

    def __init__(self, node, name, action_type):
        import threading
        from rclpy.action import ActionClient
        self._threading = threading
        self._client = ActionClient(node, action_type, name)
        self._goal_handle = None
        self._result = None
        self._done = threading.Event()
        self._done.set()

    def wait_for_server(self, timeout=None):
        return self._client.wait_for_server(timeout_sec=timeout)

    def send_goal(self, goal, feedback_cb=None):
        self._goal_handle = None
        self._result = None
        self._done.clear()

        def goal_response_cb(future):
            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self._done.set()
                return
            self._goal_handle.get_result_async().add_done_callback(result_cb)

        def result_cb(future):
            self._result = future.result().result
            self._done.set()

        kwargs = {}
        if feedback_cb is not None:
            kwargs['feedback_callback'] = lambda msg: feedback_cb(msg.feedback)
        self._client.send_goal_async(goal, **kwargs).add_done_callback(
            goal_response_cb)

    def wait_for_result(self, timeout=None):
        return self._done.wait(timeout)

    def send_goal_and_wait(self, goal, timeout=None):
        self.send_goal(goal)
        return self.wait_for_result(timeout)

    def get_result(self):
        return self._result

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


def create_simple_action_client(node, name, action_type):
    """actionlib.SimpleActionClient(name, action_type) (ROS1) or an
    equivalent wrapper of rclpy's ActionClient (ROS2). Pass `FooAction`
    under ROS1 and `Foo` under ROS2. wait_for_server(),
    wait_for_result() and send_goal_and_wait() take timeouts in seconds
    (None: wait forever) on both."""
    if ROS_VERSION == 2:
        return _SimpleActionClient(node, name, action_type)
    import actionlib

    # actionlib.SimpleActionClient is an old-style class on python2, so
    # call the base methods explicitly instead of via super()
    class _ROS1SimpleActionClient(actionlib.SimpleActionClient):
        def wait_for_server(self, timeout=None):
            return actionlib.SimpleActionClient.wait_for_server(
                self, rospy.Duration(timeout or 0))

        def wait_for_result(self, timeout=None):
            return actionlib.SimpleActionClient.wait_for_result(
                self, rospy.Duration(timeout or 0))

        def send_goal_and_wait(self, goal, timeout=None):
            self.send_goal(goal)
            return self.wait_for_result(timeout)
    return _ROS1SimpleActionClient(name, action_type)


class _SimpleActionServer(object):
    """actionlib.SimpleActionServer-like wrapper of rclpy's ActionServer,
    for the commonly used subset: execute_cb(goal) runs to completion and
    calls set_succeeded()/set_aborted()/set_preempted() once."""

    def __init__(self, node, name, action_type, execute_cb):
        from rclpy.action import ActionServer, CancelResponse
        self._action_type = action_type
        self._execute_cb = execute_cb
        self._goal_handle = None
        self._server = ActionServer(
            node, action_type, name, self._ros2_execute_cb,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT)

    def _ros2_execute_cb(self, goal_handle):
        self._goal_handle = goal_handle
        self._status, self._result = None, None
        self._execute_cb(goal_handle.request)
        if self._status == 'preempted' and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif self._status == 'succeeded':
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self._result if self._result is not None \
            else self._action_type.Result()

    def start(self):
        pass

    def is_preempt_requested(self):
        return self._goal_handle is not None and \
            self._goal_handle.is_cancel_requested

    def publish_feedback(self, feedback):
        self._goal_handle.publish_feedback(feedback)

    def set_succeeded(self, result=None, text=''):
        self._status, self._result = 'succeeded', result

    def set_aborted(self, result=None, text=''):
        self._status, self._result = 'aborted', result

    def set_preempted(self, result=None, text=''):
        self._status, self._result = 'preempted', result


def create_simple_action_server(node, name, action_type, execute_cb):
    """actionlib.SimpleActionServer(name, action_type, execute_cb,
    auto_start=False) (ROS1) or an equivalent wrapper of rclpy's
    ActionServer (ROS2); call start() on it in both. Pass `FooAction`
    under ROS1 and `Foo` under ROS2."""
    if ROS_VERSION == 2:
        return _SimpleActionServer(node, name, action_type, execute_cb)
    import actionlib
    return actionlib.SimpleActionServer(
        name, action_type, execute_cb, auto_start=False)


class _Config(dict):
    """dict with attribute access, like dynamic_reconfigure's config."""

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        self[name] = value


class _ReconfigureServer(object):

    def __init__(self, node, defaults, callback):
        from rcl_interfaces.msg import SetParametersResult
        self._result_type = SetParametersResult
        self._node = node
        self._callback = callback
        self._names = list(defaults.keys())
        for name, default in defaults.items():
            get_param(node, name, default)
        node.add_on_set_parameters_callback(self._on_set_parameters)
        self.config = self._callback(self._current(), 0)

    def _current(self):
        return _Config(
            (name, self._node.get_parameter(name).value)
            for name in self._names)

    def _on_set_parameters(self, params):
        config = self._current()
        changed = False
        for param in params:
            if param.name in config:
                config[param.name] = param.value
                changed = True
        if changed:
            self.config = self._callback(config, 0)
        return self._result_type(successful=True)


def create_reconfigure_server(node, config_type, callback, defaults=None):
    """dynamic_reconfigure.server.Server(config_type, callback) (ROS1) or
    the ROS2 equivalent built on node parameters: every key of
    `defaults` (ROS2 only, since ROS2 has no .cfg; ignored under ROS1)
    is declared as a parameter, and `callback(config, level)` is called
    once at start and on every `ros2 param set` of one of them, with
    `config` supporting both config.name and config['name'] (level is
    always 0 under ROS2). Pass config_type=None under ROS2."""
    if ROS_VERSION == 2:
        return _ReconfigureServer(node, defaults or {}, callback)
    from dynamic_reconfigure.server import Server
    return Server(config_type, callback)
