#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS1 (rosbag) / ROS2 (rosbag2_py) compatibility layer, used by
jsk_rosbag_tools.

rosbag.Bag (ROS1) and rosbag2_py's SequentialReader/SequentialWriter
(ROS2) have unrelated APIs (single .bag file vs. a sqlite3-backed
directory; deserialized messages vs. raw serialized bytes; a
rospy.Time bag-entry timestamp vs. a plain integer nanoseconds). This
module hides that behind a single rosbag.Bag-like interface
(open_bag(), .read_messages()/__iter__, .write(), get_topic_dict())
so jsk_rosbag_tools does not need ROS_VERSION branches of its own for
bag I/O.

Bag-entry timestamps (the third element yielded by read_messages(),
and the third argument to write()) are always plain integer
nanoseconds here, for both ROS1 and ROS2 -- this is what lets
arithmetic on them (as in merge.py/tf_static_to_tf.py) be
version-agnostic. Use stamp_from_nanoseconds()/nanoseconds_from_stamp()
to convert to/from a message's own Header.stamp field, which is a
distinct, version-specific message type.
"""
from __future__ import absolute_import

import collections
import os

from jsk_ros1_ros2_compat.rospy_rclpy_compat import stamp_to_sec  # NOQA: F401 (re-exported for convenience)

ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))

# Matches rosbag.Bag.read_messages()'s own BagMessage(topic, message,
# timestamp) namedtuple, so callers can use either tuple-unpacking
# (topic, msg, t) or .topic/.message/.timestamp attribute access,
# regardless of ROS_VERSION.
BagMessage = collections.namedtuple('BagMessage', ['topic', 'message', 'timestamp'])

if ROS_VERSION == 2:
    from builtin_interfaces.msg import Time as _Time
    from rclpy.serialization import deserialize_message
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
else:
    import rosbag
    import rospy


def nanoseconds_from_sec(sec):
    return int(round(sec * 1e9))


def stamp_from_nanoseconds(ns):
    """A Header.stamp-compatible value, from plain integer nanoseconds
    (the same representation used for bag-entry timestamps)."""
    ns = int(ns)
    if ROS_VERSION == 2:
        return _Time(sec=ns // 10 ** 9, nanosec=ns % 10 ** 9)
    else:
        return rospy.Time(ns // 10 ** 9, ns % 10 ** 9)


def nanoseconds_from_stamp(stamp):
    if ROS_VERSION == 2:
        return stamp.sec * 10 ** 9 + stamp.nanosec
    else:
        return stamp.to_nsec()


def message_type_name(msg):
    """'pkg/MsgName', matching ROS1's msg._type format."""
    if ROS_VERSION == 2:
        pkg = msg.__class__.__module__.split('.')[0]
        return '{}/{}'.format(pkg, msg.__class__.__name__)
    else:
        return msg._type


if ROS_VERSION == 2:
    def _short_type_name(rosbag2_type_str):
        # rosbag2 stores 'pkg/msg/Type'; normalize to ROS1's yaml-info
        # style 'pkg/Type' so callers can compare against either.
        pkg, sep, rest = rosbag2_type_str.partition('/msg/')
        return '{}/{}'.format(pkg, rest) if sep else rosbag2_type_str

    def _open_reader(path):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
        reader.open(storage_options, rosbag2_py.ConverterOptions('', ''))
        return reader

    class BagReader(object):
        """rosbag.Bag-like reader, backed by rosbag2_py.SequentialReader.

        Unlike rosbag.Bag, a SequentialReader is single-pass -- once
        exhausted it stays exhausted, it cannot be rewound. rosbag.Bag
        callers (e.g. tf_static_to_tf.py) read the same Bag object more
        than once, expecting each call to see the whole bag again, so
        read_messages()/__iter__()/get_message_count() each open their
        own short-lived SequentialReader rather than sharing one.
        """

        def __init__(self, path):
            self._path = path
            reader = _open_reader(path)
            self._type_strs = {
                t.name: t.type for t in reader.get_all_topics_and_types()}
            reader.close()
            self._msg_classes = {}

        def _msg_class(self, topic):
            if topic not in self._msg_classes:
                self._msg_classes[topic] = get_message(self._type_strs[topic])
            return self._msg_classes[topic]

        def read_messages(self, topics=None):
            if isinstance(topics, str):
                topics = [topics]
            reader = _open_reader(self._path)
            try:
                while reader.has_next():
                    topic, data, t = reader.read_next()
                    if topics is not None and topic not in topics:
                        continue
                    msg = deserialize_message(data, self._msg_class(topic))
                    yield BagMessage(topic, msg, t)
            finally:
                reader.close()

        def __iter__(self):
            return self.read_messages()

        def get_message_count(self):
            reader = _open_reader(self._path)
            count = reader.get_metadata().message_count
            reader.close()
            return count

        def close(self):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()

    class BagWriter(object):
        """rosbag.Bag-like writer, backed by rosbag2_py.SequentialWriter."""

        def __init__(self, path):
            self._writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
            self._writer.open(storage_options, rosbag2_py.ConverterOptions('', ''))
            self._known_topics = set()

        def write(self, topic, msg, t):
            if topic not in self._known_topics:
                pkg, name = message_type_name(msg).split('/')
                topic_metadata = rosbag2_py.TopicMetadata(
                    id=0, name=topic, type='{}/msg/{}'.format(pkg, name),
                    serialization_format='cdr')
                self._writer.create_topic(topic_metadata)
                self._known_topics.add(topic)
            self._writer.write(topic, serialize_message(msg), int(t))

        def close(self):
            self._writer.close()

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()

    def open_bag(path, mode='r'):
        return BagWriter(path) if mode == 'w' else BagReader(path)

    def get_topic_dict(bag_filepath):
        if not os.path.exists(bag_filepath):
            raise OSError('bag file {} not exists'.format(bag_filepath))
        reader = _open_reader(bag_filepath)
        meta = reader.get_metadata()
        reader.close()
        return {
            ti.topic_metadata.name: {
                'type': _short_type_name(ti.topic_metadata.type),
                'messages': ti.message_count,
            }
            for ti in meta.topics_with_message_count
        }

else:
    class BagReader(object):
        """rosbag.Bag-like reader; wraps ROS1's real rosbag.Bag so that
        bag-entry timestamps come out as plain integer nanoseconds,
        matching the ROS2 branch above (rosbag.Bag itself yields
        rospy.Time)."""

        def __init__(self, path):
            self._bag = rosbag.Bag(path)

        def read_messages(self, topics=None):
            for topic, msg, t in self._bag.read_messages(topics=topics):
                yield BagMessage(topic, msg, t.to_nsec())

        def __iter__(self):
            return self.read_messages()

        def get_message_count(self):
            return self._bag.get_message_count()

        def close(self):
            self._bag.close()

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()

    class BagWriter(object):
        """rosbag.Bag-like writer, taking plain integer-nanosecond
        timestamps (see BagReader above)."""

        def __init__(self, path):
            self._bag = rosbag.Bag(path, 'w')

        def write(self, topic, msg, t):
            self._bag.write(topic, msg, stamp_from_nanoseconds(t))

        def close(self):
            self._bag.close()

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            self.close()

    def open_bag(path, mode='r'):
        return BagWriter(path) if mode == 'w' else BagReader(path)

    # get_topic_dict() is not defined here under ROS1: info.py's own
    # get_topic_dict()/get_info() (backed directly by rosbag's
    # _get_yaml_info()) is the ROS1 implementation callers use.
