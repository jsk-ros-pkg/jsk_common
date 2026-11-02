"""Exercise real ROS2 discovery, remapping, QoS and publication."""
import os
import runpy
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class TestHeatmapNodeROS2(unittest.TestCase):
    def test_discovery_and_image_header(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'scripts',
                            'array_to_heatmap.py')
        cls = runpy.run_path(path)['ArrayToHeatmap']
        rclpy.init()
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            for use_image in (False, True):
                name = 'heatmap_image' if use_image else 'heatmap_array'
                args = ['--ros-args', '-r', '~/input:=/' + name + '_input',
                        '-r', '~/output:=/' + name + '_output',
                        '-p', 'cell_size:=16']
                if use_image:
                    args += ['-p', 'input_type:=sensor_msgs/msg/Image']
                node = rclpy.create_node(name, cli_args=args)
                nodes.append(node)
                executor.add_node(node)
                app = cls(node)
                peer = rclpy.create_node(name + '_peer')
                nodes.append(peer)
                executor.add_node(peer)
                messages = []
                subscription = peer.create_subscription(
                    Image, '/' + name + '_output', messages.append, 1)
                msg_type = Image if use_image else Float64MultiArray
                publisher = peer.create_publisher(
                    msg_type, '/' + name + '_input', qos_profile_sensor_data)
                if use_image:
                    msg = Image(height=1, width=2, encoding='mono8',
                                step=2, data=b'\x01\x02')
                    msg.header.frame_id = 'sensor'
                    msg.header.stamp.sec = 12
                else:
                    msg = Float64MultiArray(data=[1., 2.])
                    msg.layout.dim = [MultiArrayDimension(size=1, stride=2),
                                      MultiArrayDimension(size=2, stride=2)]
                deadline = time.monotonic() + 10
                while not messages and time.monotonic() < deadline:
                    publisher.publish(msg)
                    executor.spin_once(timeout_sec=0.05)
                self.assertTrue(messages, 'No output for ' + name)
                self.assertEqual(messages[0].encoding, 'bgr8')
                self.assertEqual((messages[0].height, messages[0].width), (16, 32))
                if use_image:
                    self.assertEqual(messages[0].header, msg.header)
                else:
                    self.assertGreater(messages[0].header.stamp.sec, 0)
                self.assertIsNotNone(app.sub)
                peer.destroy_subscription(subscription)
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()
