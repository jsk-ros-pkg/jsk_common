import os

import rosbag
import yaml


def get_info(bag_filepath):
    if not os.path.exists(bag_filepath):
        raise OSError('bag file {} not exists'.format(bag_filepath))
    info_dict = yaml.load(
        rosbag.Bag(bag_filepath)._get_yaml_info(),
        Loader=yaml.SafeLoader)
    return info_dict


def get_topic_dict(bag_filepath):
    info_dict = get_info(bag_filepath)
    topics = info_dict['topics']
    topic_dict = {}
    for topic in topics:
        topic_dict[topic['topic']] = topic
    return topic_dict
