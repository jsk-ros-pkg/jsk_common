import io
import os
import shutil
import tempfile
import unittest

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

from jsk_ros1_ros2_compat.rosbag import (
    ROS_VERSION, nanoseconds_from_stamp, open_bag, stamp_from_nanoseconds)
from jsk_rosbag_tools.add_heatmap import add_heatmap
from jsk_topic_tools.array_to_heatmap import HeatmapRenderer


def write_latched_string(bag):
    msg = String(data='keep')
    if ROS_VERSION == 1:
        stream = io.BytesIO()
        msg.serialize(stream)
        data = (String._type, stream.getvalue(), String._md5sum, String)
        metadata = {'type': String._type, 'md5sum': String._md5sum,
                    'message_definition': String._full_text, 'latching': '1'}
    else:
        import rosbag2_py
        from rosbag2_py._storage import QoS
        from rclpy.serialization import serialize_message
        data = serialize_message(msg)
        metadata = rosbag2_py.TopicMetadata(
            id=0, name='/original', type='std_msgs/msg/String',
            serialization_format='cdr',
            offered_qos_profiles=[QoS(1).transient_local()])
    bag.write_raw('/original', data, 11000000000, metadata)


class TestAddHeatmap(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.mkdtemp()
        self.source = os.path.join(self.directory, 'source.bag')
        self.output = os.path.join(self.directory, 'output.bag')
        msg = Float64MultiArray(data=[1., 2.])
        msg.layout.dim = [MultiArrayDimension(size=1, stride=2),
                          MultiArrayDimension(size=2, stride=2)]
        with open_bag(self.source, 'w') as bag:
            bag.write('/pressure', msg, 10000000020)
            write_latched_string(bag)
            image = Image(height=1, width=2, encoding='mono8', step=2,
                          data=b'\x01\x02')
            image.header.stamp = stamp_from_nanoseconds(8000000000)
            image.header.frame_id = 'image_sensor'
            bag.write('/scalar_image', image, 12000000000)

    def tearDown(self):
        shutil.rmtree(self.directory)

    def test_preservation_and_timestamp(self):
        self.assertEqual(add_heatmap(self.source, self.output, '/pressure',
                                     '/heatmap', HeatmapRenderer(cell_size=16)), 1)
        with open_bag(self.source) as src, open_bag(self.output) as dst:
            originals = list(src.read_raw_messages())
            copies = list(dst.read_raw_messages(
                topics=['/pressure', '/original', '/scalar_image']))
            for a, b in zip(originals, copies):
                self.assertEqual((a.topic, a.timestamp), (b.topic, b.timestamp))
                if ROS_VERSION == 1:
                    self.assertEqual(a.data[:3], b.data[:3])
                    self.assertEqual(a.metadata, b.metadata)
                else:
                    self.assertEqual(a.data, b.data)
                    for field in ('name', 'type', 'serialization_format',
                                  'type_description_hash'):
                        self.assertEqual(getattr(a.metadata, field),
                                         getattr(b.metadata, field))
                    # Storage-assigned IDs may change when a topic is added.
                    a.metadata.id = 0
                    b.metadata.id = 0
                    self.assertTrue(a.metadata.equals(b.metadata))
            self.assertEqual(len(originals), len(copies))
            _, image, stamp = next(dst.read_messages(topics=['/heatmap']))
            self.assertEqual(nanoseconds_from_stamp(image.header.stamp), stamp)
            self.assertEqual(stamp, 10000000020)
        with self.assertRaises(ValueError):
            add_heatmap(self.source, self.output, '/pressure', '/heatmap')

    def test_image_header(self):
        add_heatmap(self.source, self.output, '/scalar_image', '/heatmap')
        with open_bag(self.output) as bag:
            _, image, stamp = next(bag.read_messages(topics=['/heatmap']))
            self.assertEqual(nanoseconds_from_stamp(image.header.stamp), 8000000000)
            self.assertEqual(stamp, 12000000000)
            self.assertEqual(image.header.frame_id, 'image_sensor')

    def test_failure_cleanup(self):
        for source_topic, output_topic in [('/missing', '/heatmap'),
                                           ('/pressure', '/original'),
                                           ('/original', '/heatmap')]:
            with self.assertRaises(ValueError):
                add_heatmap(self.source, self.output, source_topic, output_topic)
            self.assertFalse(os.path.exists(self.output))
        self.assertEqual(os.listdir(self.directory), ['source.bag'])


if __name__ == '__main__':
    unittest.main()
