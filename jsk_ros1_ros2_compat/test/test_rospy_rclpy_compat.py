#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Tests of rospy_rclpy_compat, shared by ROS1 (rostest,
test/test_rospy_rclpy_compat.test) and ROS2 (ament_cmake_pytest)."""
import threading
import time
import unittest

from std_msgs.msg import String

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

if ROS_VERSION == 2:
    from example_interfaces.action import Fibonacci as FibonacciAction
    FibonacciGoal = FibonacciAction.Goal
    FibonacciFeedback = FibonacciAction.Feedback
    FibonacciResult = FibonacciAction.Result
else:
    import rospy
    from actionlib_tutorials.msg import FibonacciAction
    from actionlib_tutorials.msg import FibonacciFeedback
    from actionlib_tutorials.msg import FibonacciGoal
    from actionlib_tutorials.msg import FibonacciResult


def wait_until(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


class TestRospyRclpyCompat(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if ROS_VERSION == 2:
            from rclpy.executors import MultiThreadedExecutor
            ros_compat.rclpy.init()
            cls.node = ros_compat.rclpy.create_node(
                'test_rospy_rclpy_compat', namespace='/compat_test')
            cls.executor = MultiThreadedExecutor()
            cls.executor.add_node(cls.node)
            cls.spin_thread = threading.Thread(target=cls.executor.spin)
            cls.spin_thread.daemon = True
            cls.spin_thread.start()
        else:
            cls.node = None
            rospy.init_node('test_rospy_rclpy_compat')

    @classmethod
    def tearDownClass(cls):
        if ROS_VERSION == 2:
            cls.executor.shutdown()
            cls.node.destroy_node()
            ros_compat.rclpy.try_shutdown()

    def test_get_param(self):
        self.assertEqual(
            ros_compat.get_param(self.node, '~not_set', 'default'), 'default')
        self.assertEqual(ros_compat.get_param(self.node, '~not_set_int', 3), 3)

    def test_get_name(self):
        self.assertTrue(
            ros_compat.get_name(self.node).endswith('test_rospy_rclpy_compat'))

    def test_logging(self):
        # only checks that every level/throttle helper accepts a str
        for log in (ros_compat.logdebug, ros_compat.loginfo,
                    ros_compat.logwarn, ros_compat.logerr,
                    ros_compat.logfatal):
            log(self.node, 'test message')
        for log in (ros_compat.loginfo_throttle, ros_compat.logwarn_throttle,
                    ros_compat.logerr_throttle):
            for _ in range(3):
                log(self.node, 10.0, 'test throttled message')

    def test_pubsub(self):
        received = []
        sub = ros_compat.create_subscription(
            self.node, 'test_pubsub', String, lambda msg: received.append(msg.data))
        pub = ros_compat.create_publisher(self.node, 'test_pubsub', String)
        self.assertTrue(wait_until(
            lambda: (pub.get_subscription_count() if ROS_VERSION == 2
                     else pub.get_num_connections()) > 0))
        for _ in range(10):
            pub.publish(String(data='hello'))
            if wait_until(lambda: received, timeout=0.5):
                break
        self.assertEqual(received[0], 'hello')
        if ROS_VERSION == 1:
            # queue_size=None keeps rospy's unbounded default, and an
            # explicit queue_size is passed through (it used to be dropped)
            self.assertIsNone(sub.impl.queue_size)
            sub300 = ros_compat.create_subscription(
                self.node, 'test_pubsub_300', String, lambda msg: None, 300)
            self.assertEqual(sub300.impl.queue_size, 300)

    def test_action(self):
        def execute_cb(goal):
            sequence = [0, 1]
            for i in range(1, goal.order):
                if server.is_preempt_requested():
                    server.set_preempted()
                    return
                sequence.append(sequence[i] + sequence[i - 1])
                feedback = FibonacciFeedback()
                feedback.sequence = sequence
                server.publish_feedback(feedback)
                time.sleep(0.05)
            result = FibonacciResult()
            result.sequence = sequence
            server.set_succeeded(result)
        server = ros_compat.create_simple_action_server(
            self.node, 'test_fibonacci', FibonacciAction, execute_cb)
        server.start()
        client = ros_compat.create_simple_action_client(
            self.node, 'test_fibonacci', FibonacciAction)
        self.assertTrue(client.wait_for_server(10.0))

        goal = FibonacciGoal()
        goal.order = 6
        self.assertTrue(client.send_goal_and_wait(goal, 10.0))
        self.assertEqual(list(client.get_result().sequence),
                         [0, 1, 1, 2, 3, 5, 8])

        feedbacks = []
        goal.order = 100
        client.send_goal(goal, feedback_cb=lambda fb: feedbacks.append(fb))
        self.assertTrue(wait_until(lambda: feedbacks))
        client.cancel_goal()
        self.assertTrue(client.wait_for_result(10.0))
        self.assertLess(len(feedbacks), 99)

    @unittest.skipIf(ROS_VERSION == 1, 'ROS1 is dynamic_reconfigure itself')
    def test_reconfigure_server(self):
        from rclpy.parameter import Parameter
        configs = []

        def callback(config, level):
            configs.append(dict(config))
            config.rate = config.rate * 2  # callbacks may modify config
            return config
        server = ros_compat.create_reconfigure_server(
            self.node, None, callback, {'rate': 1.0, 'frame_id': 'map'})
        self.assertEqual(configs, [{'rate': 1.0, 'frame_id': 'map'}])
        self.assertEqual(server.config.rate, 2.0)
        self.node.set_parameters([Parameter('rate', value=2.5)])
        self.assertEqual(configs[-1], {'rate': 2.5, 'frame_id': 'map'})
        self.assertEqual(server.config['rate'], 5.0)
        # unrelated parameters do not call the callback
        self.node.declare_parameter('unrelated', 0)
        self.node.set_parameters([Parameter('unrelated', value=1)])
        self.assertEqual(len(configs), 2)


class TestShutdown(unittest.TestCase):
    """on_shutdown/signal_shutdown (and, on ROS2, spin_and_shutdown()
    returning) need their own process-wide shutdown, so run last."""

    @unittest.skipIf(ROS_VERSION == 1, 'shuts rospy down for the other tests')
    def test_signal_shutdown_ros2(self):
        ros_compat.rclpy.init()
        node = ros_compat.rclpy.create_node('test_signal_shutdown')
        called = []
        ros_compat.on_shutdown(node, lambda: called.append(True))
        ros_compat.create_timer(
            node, 0.2, lambda event: ros_compat.signal_shutdown(node, 'test'))
        thread = threading.Thread(target=ros_compat.spin_and_shutdown, args=(node,))
        thread.start()
        thread.join(10.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(called, [True])


if __name__ == '__main__':
    import rostest
    rostest.rosrun('jsk_ros1_ros2_compat', 'test_rospy_rclpy_compat',
                   TestRospyRclpyCompat)
