"""Exercise jsk_topic_tools.ConnectionBasedTransport on ROS2: subscribe()
with the first subscriber of ~output, unsubscribe() without any, and
~always_subscribe."""
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from jsk_topic_tools import ConnectionBasedTransport


class Relay(ConnectionBasedTransport):

    def __init__(self, node):
        super(Relay, self).__init__(node)
        self.sub = None
        self.pub = self.advertise('~output', String, queue_size=1)

    def subscribe(self):
        self.sub = self.node.create_subscription(
            String, '~/input', self.pub.publish, 1)

    def unsubscribe(self):
        self.node.destroy_subscription(self.sub)
        self.sub = None


class TestConnectionBasedTransportROS2(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()

    def create_node(self, name, **kwargs):
        node = rclpy.create_node(name, **kwargs)
        self.executor.add_node(node)
        self.nodes.append(node)
        return node

    def spin_until(self, predicate, seconds=5.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not predicate():
            self.executor.spin_once(timeout_sec=0.05)
        return predicate()

    def test_subscribe_with_subscribers(self):
        relay = Relay(self.create_node('relay'))
        self.assertFalse(relay.is_subscribed())
        self.assertIsNone(relay.sub)

        client = self.create_node('client')
        received = []
        sub = client.create_subscription(
            String, '/relay/output', received.append, 1)
        self.assertTrue(self.spin_until(relay.is_subscribed))
        pub = client.create_publisher(String, '/relay/input', 1)
        self.assertTrue(self.spin_until(
            lambda: pub.get_subscription_count() > 0))
        self.assertTrue(self.spin_until(
            lambda: pub.publish(String(data='hello')) or received))
        self.assertEqual(received[0].data, 'hello')

        client.destroy_subscription(sub)
        self.assertTrue(self.spin_until(lambda: not relay.is_subscribed()))
        self.assertIsNone(relay.sub)

    def test_always_subscribe(self):
        relay = Relay(self.create_node(
            'relay', parameter_overrides=[
                Parameter('always_subscribe', value=True)]))
        self.assertTrue(relay.is_subscribed())
        self.assertIsNotNone(relay.sub)

        client = self.create_node('client')
        sub = client.create_subscription(
            String, '/relay/output', lambda msg: None, 1)
        self.assertTrue(self.spin_until(
            lambda: relay.pub.get_subscription_count() > 0))
        client.destroy_subscription(sub)
        self.assertTrue(self.spin_until(
            lambda: relay.pub.get_subscription_count() == 0))
        self.spin_until(lambda: False, 0.5)
        self.assertTrue(relay.is_subscribed())

    def test_no_publishers(self):
        class NoPublisher(ConnectionBasedTransport):
            def subscribe(self):
                pass

            def unsubscribe(self):
                pass

        with self.assertRaises(RuntimeError):
            NoPublisher(self.create_node('no_publisher'))


if __name__ == '__main__':
    unittest.main()
