"""Add a heatmap topic to ROS 1/ROS 2 bags."""

import os
import shutil
import tempfile

from jsk_ros1_ros2_compat.rosbag import open_bag, stamp_from_nanoseconds
from jsk_rosbag_tools.info import get_topic_dict
from jsk_topic_tools.array_to_heatmap import HeatmapRenderer


def add_heatmap(input_path, output_path, input_topic, output_topic,
                renderer=None, rows=0, cols=0, frame_id=''):
    """Copy serialized messages, metadata and nanosecond recording timestamps.

    The compat layer selects ROS1 files or ROS2 directories from ROS_VERSION.
    Only the selected input topic is deserialized. Failures leave no destination.
    """
    input_path = os.path.abspath(input_path)
    output_path = os.path.abspath(output_path)
    if os.path.lexists(output_path):
        raise ValueError('Output already exists: ' + output_path)
    topics = get_topic_dict(input_path)
    if input_topic not in topics:
        raise ValueError('Input topic not found: ' + input_topic)
    if output_topic in topics or input_topic == output_topic:
        raise ValueError('Output topic already exists: ' + output_topic)
    renderer = renderer or HeatmapRenderer()
    temporary = tempfile.mkdtemp(prefix='.heatmap-',
                                 dir=os.path.dirname(output_path))
    bag_path = os.path.join(temporary, 'bag')
    count = 0
    try:
        with open_bag(input_path) as source, open_bag(bag_path, 'w') as destination:
            for topic, data, stamp, metadata in source.read_raw_messages():
                destination.write_raw(topic, data, stamp, metadata)
                if topic == input_topic:
                    msg = source.deserialize_raw(topic, data)
                    image = renderer.to_image(
                        msg, stamp_from_nanoseconds(stamp), frame_id, rows, cols)
                    destination.write(output_topic, image, stamp)
                    count += 1
        if os.path.isdir(bag_path):
            # Reserve the destination exclusively, then replace our empty directory
            # with the completed bag. An existing destination is never replaced.
            os.mkdir(output_path)
            try:
                os.rename(bag_path, output_path)
            except Exception:
                os.rmdir(output_path)
                raise
        else:
            os.link(bag_path, output_path)
    finally:
        shutil.rmtree(temporary)
    return count
