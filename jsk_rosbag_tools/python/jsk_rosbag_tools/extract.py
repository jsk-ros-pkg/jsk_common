import numpy as np
import rosbag

from jsk_rosbag_tools.cv import compressed_format
from jsk_rosbag_tools.cv import decompresse_imgmsg
from jsk_rosbag_tools.cv import msg_to_img
from jsk_rosbag_tools.info import get_topic_dict


def get_image_topic_names(bag_filepath,
                          rgb_only=False):
    topic_dict = get_topic_dict(bag_filepath)
    topic_names = []
    for topic_name, info in topic_dict.items():
        if info['type'] == 'sensor_msgs/Image' or \
                info['type'] == 'sensor_msgs/CompressedImage':
            if rgb_only:
                msg = extract_oneshot_topic(bag_filepath, topic_name)
                if topic_dict[topic_name]['type'] == 'sensor_msgs/Image':
                    encoding = msg.encoding
                else:
                    encoding, _ = compressed_format(msg)
                if encoding in ['bgra8', 'bgr8',
                                'rgba8', 'rgb8']:
                    topic_names.append(topic_name)
            else:
                topic_names.append(topic_name)
    return topic_names


def extract_oneshot_topic(bag_filepath, topic_name):
    topic_dict = get_topic_dict(bag_filepath)

    if topic_name not in topic_dict:
        raise ValueError('Topic not exists {}'.
                         format(topic_name))

    msg = None
    with rosbag.Bag(bag_filepath, 'r') as input_rosbag:
        for topic, msg, stamp in input_rosbag.read_messages(
                topics=[topic_name, ]):
            break
    return msg


def extract_image_topic(bag_filepath, topic_name):
    topic_dict = get_topic_dict(bag_filepath)
    if topic_name not in topic_dict:
        raise ValueError("topic ({}) is not included in bagfile ({})."
                         .format(topic_name, bag_filepath))

    with rosbag.Bag(bag_filepath, 'r') as input_rosbag:
        for topic, msg, _ in input_rosbag.read_messages(
                topics=[topic_name]):
            topic_type = topic_dict[topic]['type']
            if topic_type == 'sensor_msgs/Image':
                bgr_img = msg_to_img(msg)
                encoding = msg.encoding
            elif topic_type == 'sensor_msgs/CompressedImage':
                bgr_img = decompresse_imgmsg(msg)
                encoding, _ = compressed_format(msg)
            else:
                raise RuntimeError('Unsupported Image topic {}'.format(
                    topic_type))

            # padding image
            if bgr_img.shape[0] % 2 != 0:
                if bgr_img.ndim == 2:
                    pad_img = np.zeros(
                        (1, bgr_img.shape[1]), dtype=bgr_img.dtype)
                elif bgr_img.ndim == 3:
                    pad_img = np.zeros(
                        (1, bgr_img.shape[1], bgr_img.shape[2]),
                        dtype=bgr_img.dtype)
                else:
                    raise ValueError('Invalid image shape {}'
                                     .format(bgr_img.shape))
                bgr_img = np.concatenate([bgr_img, pad_img], axis=0)

            if bgr_img.shape[1] % 2 != 0:
                if bgr_img.ndim == 2:
                    pad_img = np.zeros(
                        (bgr_img.shape[0], 1), dtype=bgr_img.dtype)
                elif bgr_img.ndim == 3:
                    pad_img = np.zeros(
                        (bgr_img.shape[0], 1, bgr_img.shape[2]),
                        dtype=bgr_img.dtype)
                else:
                    raise ValueError('Invalid image shape {}'
                                     .format(bgr_img.shape))
                bgr_img = np.concatenate([bgr_img, pad_img], axis=1)

            yield msg.header.stamp.to_sec(), topic, bgr_img, encoding
