"""Exercise filtered_relay.py on ROS2: discovery, ~filter, ~transform."""
import os
import runpy
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String


class TestFilteredRelayROS2(unittest.TestCase):

    def spin(self, executor, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    def run_relay(self, name, params, in_type, out_type, inputs):
        """Publish inputs to a filtered_relay node of params, return what it
        relays."""
        path = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                            'filtered_relay.py')
        cls = runpy.run_path(path)['FilteredRelay']
        rclpy.init()
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            args = ['--ros-args', '-r', '~/input:=/' + name + '_input',
                    '-r', '~/output:=/' + name + '_output']
            for param in params:
                args += ['-p', param]
            node = rclpy.create_node(name, cli_args=args)
            nodes.append(node)
            executor.add_node(node)
            cls(node)
            peer = rclpy.create_node(name + '_peer')
            nodes.append(peer)
            executor.add_node(peer)
            messages = []
            peer.create_subscription(
                out_type, '/' + name + '_output', messages.append, 10)
            publisher = peer.create_publisher(in_type, '/' + name + '_input', 10)
            # until the node discovers the input and the output is connected
            deadline = time.monotonic() + 10
            while not messages and time.monotonic() < deadline:
                publisher.publish(inputs[-1])
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(messages, 'No output for ' + name)
            self.spin(executor, 0.5)  # the rest of the warm-up messages
            del messages[:]
            for msg in inputs:
                publisher.publish(msg)
            self.spin(executor, 1.0)
            return messages
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()

    def test_filter_and_transform(self):
        # discovered input type, output built from the transform result
        messages = self.run_relay(
            'filtered_relay_string',
            ["filter:=m.data.startswith('hello')",
             'transform:=m.data.upper()',
             'output_type:=std_msgs/String'],
            String, String,
            [String(data='bye'), String(data='hello world')])
        self.assertEqual([m.data for m in messages], ['HELLO WORLD'])

    def test_import(self):
        # ~import modules in the expressions
        messages = self.run_relay(
            'filtered_relay_numpy',
            ["import:=['numpy']",
             'filter:=numpy.abs(m.data) > 1.0',
             'transform:=float(numpy.abs(m.data))',
             'output_type:=std_msgs/Float32'],
            Float64, Float32,
            [Float64(data=0.5), Float64(data=-2.0)])
        self.assertEqual([m.data for m in messages], [2.0])

    def test_relay_without_transform(self):
        messages = self.run_relay(
            'filtered_relay_plain',
            ['output_type:=std_msgs/msg/String'],
            String, String,
            [String(data='a'), String(data='b')])
        self.assertEqual([m.data for m in messages], ['a', 'b'])
