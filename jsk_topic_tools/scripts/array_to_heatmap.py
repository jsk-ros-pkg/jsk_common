#!/usr/bin/env python
"""Publish a scalar MultiArray or Image as a labeled heatmap (ROS 1 and ROS 2)."""

import cv2
from sensor_msgs.msg import Image

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros
from jsk_topic_tools.array_to_heatmap import HeatmapRenderer


class ArrayToHeatmap(object):
    def __init__(self, node=None):
        self.node = node
        self.rows = int(ros.get_param(node, '~rows', 0))
        self.cols = int(ros.get_param(node, '~cols', 0))
        self.frame_id = ros.get_param(node, '~frame_id', '')
        self.renderer = HeatmapRenderer(**{
            name: ros.get_param(node, '~' + name, default) for name, default in (
                ('cell_size', 64), ('show_values', True), ('precision', 2),
                ('colormap', 'jet'), ('min_value', None), ('max_value', None))})
        self.pub = ros.create_publisher(node, '~output', Image, queue_size=1)
        self.sub = None
        input_type = ros.get_param(node, '~input_type', '')
        if input_type:
            self.subscribe(input_type)
        elif ros.ROS_VERSION == 1:
            import rospy
            self.sub = ros.create_subscription(
                node, '~input', rospy.AnyMsg, self.raw_callback, queue_size=1)
        else:
            # ROS2 needs a concrete message class. Poll discovery without blocking
            # startup, so publishers may appear after this node starts.
            self.discovery_timer = ros.create_timer(node, 0.5, self.discover)

    def subscribe(self, type_name):
        cls = ros.resolve_message_class(type_name)
        if cls is None:
            raise ValueError('Input message class is not installed: ' + type_name)
        if ros.ROS_VERSION == 2:
            from rclpy.qos import qos_profile_sensor_data
            # Best-effort input accepts both reliable and sensor-data publishers.
            self.sub = self.node.create_subscription(
                cls, '~/input', self.callback, qos_profile_sensor_data)
        else:
            self.sub = ros.create_subscription(
                self.node, '~input', cls, self.callback, queue_size=1)

    def discover(self, event):
        if self.sub is not None:
            return
        topic = self.node.resolve_topic_name('~/input')
        types = dict(self.node.get_topic_names_and_types()).get(topic, [])
        if len(types) != 1:
            return
        try:
            self.subscribe(types[0])
            self.discovery_timer.cancel()
        except (ValueError, ImportError, AttributeError) as exc:
            ros.logwarn_throttle(self.node, 5.0, 'Cannot subscribe: %s' % exc)

    def raw_callback(self, raw):
        stamp = ros.stamp_now(self.node)
        try:
            cls = ros.resolve_message_class(raw._connection_header['type'])
            if cls is None:
                raise ValueError('Input message class is not installed')
            msg = cls().deserialize(raw._buff)
            self.publish(msg, stamp)
        except (ValueError, TypeError, cv2.error) as exc:
            ros.logwarn_throttle(self.node, 5.0, 'Cannot render heatmap: %s' % exc)

    def callback(self, msg):
        stamp = ros.stamp_now(self.node)
        try:
            self.publish(msg, stamp)
        except (ValueError, TypeError, cv2.error) as exc:
            ros.logwarn_throttle(self.node, 5.0, 'Cannot render heatmap: %s' % exc)

    def publish(self, msg, stamp):
        self.pub.publish(self.renderer.to_image(
            msg, stamp, self.frame_id, self.rows, self.cols))


if __name__ == '__main__':
    node = ros.init_node('array_to_heatmap')
    app = ArrayToHeatmap(node)
    ros.spin(node)
