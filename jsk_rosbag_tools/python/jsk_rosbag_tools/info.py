import os

from jsk_ros1_ros2_compat.rosbag import ROS_VERSION

if ROS_VERSION == 2:
    from jsk_ros1_ros2_compat.rosbag import get_topic_dict as _get_topic_dict_ros2
else:
    import rosbag
    import yaml


def get_info(bag_filepath):
    """ROS1 only: the full rosbag yaml-info dict."""
    if ROS_VERSION == 2:
        raise NotImplementedError(
            'get_info() relies on rosbag (ROS1)\'s yaml info; '
            'use get_topic_dict() instead under ROS2.')
    if not os.path.exists(bag_filepath):
        raise OSError('bag file {} not exists'.format(bag_filepath))
    info_dict = yaml.load(
        rosbag.Bag(bag_filepath)._get_yaml_info(),
        Loader=yaml.SafeLoader)
    return info_dict


def get_topic_dict(bag_filepath):
    if ROS_VERSION == 2:
        return _get_topic_dict_ros2(bag_filepath)
    info_dict = get_info(bag_filepath)
    topics = info_dict['topics']
    topic_dict = {}
    for topic in topics:
        topic_dict[topic['topic']] = topic
    return topic_dict
