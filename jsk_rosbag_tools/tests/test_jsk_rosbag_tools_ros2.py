#!/usr/bin/env python
"""ROS2-only pytest coverage for the jsk_ros1_ros2_compat.rosbag-backed
bag I/O layer.

Unlike the ROS1 rostest suite (test_jsk_rosbag_tools.py), which
downloads sample .bag files and runs the CLI scripts as subprocesses,
this records a couple of tiny bags of its own with jsk_ros1_ros2_compat.
rosbag's BagWriter -- ROS2 rosbag2 bags are a different (sqlite3-backed
directory), incompatible container format, so the ROS1 sample data
cannot be reused here.
"""
import os
import shutil
import tempfile

import cv_bridge
import numpy as np
from std_msgs.msg import String

from jsk_rosbag_tools.compress import compress_bag_imgs
from jsk_rosbag_tools.extract import extract_image_topic
from jsk_rosbag_tools.extract import extract_oneshot_topic
from jsk_rosbag_tools.extract import get_image_topic_names
from jsk_rosbag_tools.info import get_topic_dict
from jsk_rosbag_tools.merge import merge_bag
from jsk_ros1_ros2_compat.rosbag import nanoseconds_from_sec
from jsk_ros1_ros2_compat.rosbag import nanoseconds_from_stamp
from jsk_ros1_ros2_compat.rosbag import open_bag
from jsk_ros1_ros2_compat.rosbag import stamp_from_nanoseconds
from jsk_ros1_ros2_compat.rosbag import stamp_to_sec


def _make_image_msg(bridge, stamp_ns):
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[:, :, 2] = 255  # solid red, so encoding round-trips visibly.
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    msg.header.stamp = stamp_from_nanoseconds(stamp_ns)
    msg.header.frame_id = 'camera'
    return msg


def _write_sample_bag(path, n_frames=3, start_ns=1700000000000000000):
    bridge = cv_bridge.CvBridge()
    with open_bag(path, 'w') as bag:
        for i in range(n_frames):
            ns = start_ns + nanoseconds_from_sec(i * 0.1)
            bag.write('/camera/image', _make_image_msg(bridge, ns), ns)
            info = String(data='frame {}'.format(i))
            bag.write('/status', info, ns)


class TestJSKRosBagToolsROS2(object):

    def setup_method(self):
        self.tmpdir = tempfile.mkdtemp(prefix='jsk_rosbag_tools_test_')

    def teardown_method(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_stamp_helpers_roundtrip(self):
        ns = nanoseconds_from_sec(12.5)
        assert ns == 12500000000
        stamp = stamp_from_nanoseconds(ns)
        assert nanoseconds_from_stamp(stamp) == ns
        assert abs(stamp_to_sec(stamp) - 12.5) < 1e-6

    def test_get_topic_dict(self):
        bag_path = os.path.join(self.tmpdir, 'sample')
        _write_sample_bag(bag_path)
        topic_dict = get_topic_dict(bag_path)
        assert topic_dict['/camera/image']['type'] == 'sensor_msgs/Image'
        assert topic_dict['/camera/image']['messages'] == 3
        assert topic_dict['/status']['type'] == 'std_msgs/String'

    def test_extract(self):
        bag_path = os.path.join(self.tmpdir, 'sample')
        _write_sample_bag(bag_path)

        assert get_image_topic_names(bag_path) == ['/camera/image']

        msg = extract_oneshot_topic(bag_path, '/status')
        assert msg.data == 'frame 0'

        results = list(extract_image_topic(bag_path, '/camera/image'))
        assert len(results) == 3
        stamp_sec, topic, bgr_img, encoding = results[0]
        assert topic == '/camera/image'
        assert encoding == 'bgr8'
        assert bgr_img.shape[:2] == (4, 4)

    def test_compress(self):
        bag_path = os.path.join(self.tmpdir, 'sample')
        out_path = os.path.join(self.tmpdir, 'compressed')
        _write_sample_bag(bag_path)

        compress_bag_imgs(bag_path, out_path, show_progress_bar=False)

        topic_dict = get_topic_dict(out_path)
        assert '/camera/image/compressed' in topic_dict
        assert topic_dict['/camera/image/compressed']['type'] == \
            'sensor_msgs/CompressedImage'

    def test_merge(self):
        bag_a = os.path.join(self.tmpdir, 'a')
        bag_b = os.path.join(self.tmpdir, 'b')
        out_path = os.path.join(self.tmpdir, 'merged')
        _write_sample_bag(bag_a, n_frames=2, start_ns=1700000000000000000)
        _write_sample_bag(bag_b, n_frames=2, start_ns=1700000001000000000)

        merge_bag(bag_a, bag_b, outfile=out_path, reindex=False)

        topic_dict = get_topic_dict(out_path)
        assert topic_dict['/camera/image']['messages'] == 4
        assert topic_dict['/status']['messages'] == 4

    def test_bag_reader_is_rereadable(self):
        # Mirrors tf_static_to_tf.py's usage: read_messages()/__iter__()
        # each called more than once on the same open_bag() object,
        # expecting to see the whole bag again each time (unlike a raw
        # rosbag2_py.SequentialReader, which is single-pass).
        bag_path = os.path.join(self.tmpdir, 'sample')
        _write_sample_bag(bag_path, n_frames=2)

        bag = open_bag(bag_path)
        first_pass = list(bag.read_messages(topics=['/status']))
        assert len(first_pass) == 2
        second_pass = list(bag)
        assert len(second_pass) == 4  # both topics, 2 frames each
        assert bag.get_message_count() == 4
