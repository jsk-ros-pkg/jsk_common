#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import os.path as osp
import pickle as pkl
import subprocess
import signal
import sys

import numpy as np
import PIL.Image
import yaml

import cv_bridge
import message_filters

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

if ROS_VERSION == 2:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rosidl_runtime_py import message_to_yaml
    from std_srvs.srv import Trigger
else:
    import genpy
    import rospy
    from jsk_topic_tools.log_utils import jsk_logfatal
    from std_srvs.srv import Trigger


def dump_ndarray(filename, arr):
    ext = osp.splitext(filename)[1]
    if ext == '.pkl':
        pkl.dump(arr, open(filename, 'wb'))
    elif ext == '.npz':
        np.savez_compressed(filename, arr)
    elif ext in ['.png', '.jpg']:
        PIL.Image.fromarray(arr).save(filename)
    else:
        raise ValueError


class DataCollectionServer(object):
    """Server to collect data.

    Under ROS1, ~topics/~params are a list-of-dict rosparam (see the
    class docstring example below). ROS2 parameters can't hold nested
    structures, so under ROS2 the same table is expressed as parallel
    string-array parameters: topics_name/topics_msg_class/topics_fname/
    topics_savetype and params_key/params_fname/params_savetype.

      <rosparam>
        save_dir: ~/.ros
        topics:
          - name: /camera/rgb/image_raw
            msg_class: sensor_msgs/Image
            fname: image.png
            savetype: ColorImage
          - name: /camera/depth/image_raw
            msg_class: sensor_msgs/Image
            fname: depth.pkl
            savetype: DepthImage
        params:
          - key: /in_hand_data_collection_main/object
            fname: label.txt
            savetype: Text
      </rosparam>
    """

    def __init__(self, node=None):
        self.node = node

        if ROS_VERSION == 2:
            self._declare_ros2_parameters()
            node.add_on_set_parameters_callback(self._on_set_parameters_ros2)
            self._update_save_dir(ros_compat.get_param(node, '~save_dir', '~/.ros'))
            self.topics = self._load_topics_ros2()
            self.params = self._load_params_ros2()
        else:
            import dynamic_reconfigure.server
            from jsk_data.cfg import DataCollectionServerConfig
            dynamic_reconfigure.server.Server(
                DataCollectionServerConfig, self.reconfig_cb)
            self.topics = ros_compat.get_param(node, '~topics', [])
            for topic in self.topics:
                self._validate_fields(topic, ['name', 'msg_class', 'fname', 'savetype'])
            self.params = ros_compat.get_param(node, '~params', [])
            for param in self.params:
                self._validate_fields(param, ['key', 'fname', 'savetype'])

        self.msg = {}
        self.slop = ros_compat.get_param(node, '~slop', 0.1)

        method = ros_compat.get_param(node, '~method', 'request')
        if method not in ['request', 'timer', 'all', 'message_filters']:
            raise ValueError('Unexpected method: {}'.format(method))
        use_message_filters = ros_compat.get_param(node, '~message_filters', False)
        self.timestamp_save_dir = ros_compat.get_param(node, '~timestamp_save_dir', True)
        self.wait_timer = ros_compat.get_param(node, '~wait_timer', False)
        self.wait_save_request = ros_compat.get_param(node, '~wait_save_request', False)
        self.rosbag_topics = [t for t in ros_compat.get_param(node, '~rosbag_topics', []) if t]
        if self.rosbag_topics:
            self.rosbag_process = None
            self.rosbag_prefix = ros_compat.get_param(node, '~rosbag_prefix', 'rosbag')
            self.rosbag_options = ros_compat.get_param(node, '~rosbag_options', '').split()

        if ROS_VERSION == 1:
            if ros_compat.has_param(node, '~with_request'):
                ros_compat.logwarn(node, 'Deprecated param: ~with_request, Use ~method')
                if not ros_compat.get_param(node, '~with_request'):
                    use_message_filters = True
                    method = 'all'
            if method == 'message_filters':
                ros_compat.logwarn(
                    node, 'Deprecated param: ~method: message_filters,'
                    'Use ~message_filters: true')
                use_message_filters = True
                method = 'all'

        # set subscribers
        self.subs = []
        for topic in self.topics:
            msg_class = ros_compat.resolve_message_class(topic['msg_class'])
            if use_message_filters:
                if ROS_VERSION == 2:
                    sub = message_filters.Subscriber(node, msg_class, topic['name'])
                else:
                    sub = message_filters.Subscriber(topic['name'], msg_class)
            else:
                sub = ros_compat.create_subscription(
                    node, topic['name'], msg_class,
                    lambda msg, name=topic['name']: self.sub_cb(msg, name), 1)
            self.subs.append(sub)

        # add synchoronizer if use_message_filters
        if use_message_filters:
            queue_size = ros_compat.get_param(node, '~queue_size', 10)
            approximate_sync = ros_compat.get_param(node, '~approximate_sync', False)
            if approximate_sync:
                slop = ros_compat.get_param(node, '~slop', 0.1)
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    self.subs, queue_size=queue_size, slop=slop)
            else:
                self.sync = message_filters.TimeSynchronizer(
                    self.subs, queue_size=queue_size)

        # set collecting method
        if method == 'request':
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_cb)
                self.server = ros_compat.create_trigger_service(
                    node, '~save_request', Trigger, self.sync_service_cb)
            else:
                self.server = ros_compat.create_trigger_service(
                    node, '~save_request', Trigger, self.service_cb)
        elif method == 'timer':
            duration = 1.0 / ros_compat.get_param(node, '~hz', 1.0)
            self.start = False
            self.start_server = ros_compat.create_trigger_service(
                node, '~start_request', Trigger, self.start_service_cb)
            self.end_server = ros_compat.create_trigger_service(
                node, '~end_request', Trigger, self.end_service_cb)
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_cb)
                self.timer = ros_compat.create_timer(node, duration, self.sync_timer_cb)
            else:
                self.timer = ros_compat.create_timer(node, duration, self.timer_cb)
        else:
            assert method == 'all'
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_and_save_cb)
            else:
                ros_compat.logerr(
                    node, '~use_filters: False, ~method: all is not supported')
                sys.exit(1)

    def _validate_fields(self, entry, required_fields):
        for field in required_fields:
            if field not in entry:
                jsk_logfatal("Required field '{}' is missing".format(field))
                sys.exit(1)

    # -- ROS2 parameter handling (see class docstring for why this
    # differs from ROS1's ~topics/~params list-of-dict rosparam) --

    def _declare_ros2_parameters(self):
        node = self.node
        node.declare_parameter('save_dir', '~/.ros')
        for name in ('topics_name', 'topics_msg_class', 'topics_fname', 'topics_savetype',
                     'params_key', 'params_fname', 'params_savetype', 'rosbag_topics'):
            node.declare_parameter(name, [''])
        node.declare_parameter('method', 'request')
        node.declare_parameter('message_filters', False)
        node.declare_parameter('timestamp_save_dir', True)
        node.declare_parameter('wait_timer', False)
        node.declare_parameter('wait_save_request', False)
        node.declare_parameter('hz', 1.0)
        node.declare_parameter('slop', 0.1)
        node.declare_parameter('queue_size', 10)
        node.declare_parameter('approximate_sync', False)
        node.declare_parameter('rosbag_prefix', 'rosbag')
        node.declare_parameter('rosbag_options', '')

    def _load_table_ros2(self, name_key, *value_keys_and_fields):
        # value_keys_and_fields is (key2, key3, ..., fields_tuple).
        value_keys = value_keys_and_fields[:-1]
        fields = value_keys_and_fields[-1]
        columns = [self.node.get_parameter(name_key).value]
        for key in value_keys:
            columns.append(self.node.get_parameter(key).value)
        table = []
        for row in zip(*columns):
            if not row[0]:
                continue
            table.append(dict(list(zip(fields, row))))
        return table

    def _load_topics_ros2(self):
        return self._load_table_ros2(
            'topics_name', 'topics_msg_class', 'topics_fname', 'topics_savetype',
            ('name', 'msg_class', 'fname', 'savetype'))

    def _load_params_ros2(self):
        return self._load_table_ros2(
            'params_key', 'params_fname', 'params_savetype',
            ('key', 'fname', 'savetype'))

    def _update_save_dir(self, value):
        self.save_dir = osp.expanduser(value).rstrip()
        if not osp.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def _on_set_parameters_ros2(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'save_dir':
                self._update_save_dir(param.value)
        return SetParametersResult(successful=True)

    def reconfig_cb(self, config, level):
        # ROS1 dynamic_reconfigure callback for ~save_dir.
        self._update_save_dir(config['save_dir'])
        return config

    def __del__(self):
        if ROS_VERSION == 1:
            for sub in getattr(self, 'subs', []):
                sub.unregister()

    def sync_sub_cb(self, *msgs):
        for topic, msg in zip(self.topics, msgs):
            self.msg[topic['name']] = {
                'stamp': msg.header.stamp,
                'msg': msg
            }

    def sync_sub_and_save_cb(self, *msgs):
        self.sync_sub_cb(*msgs)
        self._sync_save()

    def sub_cb(self, msg, topic_name):
        if ROS_VERSION == 2:
            stamp = msg.header.stamp if hasattr(msg, 'header') else ros_compat.stamp_now(self.node)
        else:
            stamp = msg.header.stamp if msg._has_header else rospy.Time.now()
        self.msg[topic_name] = {'stamp': stamp, 'msg': msg}

    def start_rosbag(self):
        postfix = ros_compat.stamp_now(self.node)
        filename = os.path.join(
            self.save_dir, self.rosbag_prefix + '-{0}'.format(postfix))
        if ROS_VERSION == 2:
            cmd_rosbag = ['ros2', 'bag', 'record']
            cmd_rosbag.extend(self.rosbag_topics)
            cmd_rosbag.extend(['-o', filename])
        else:
            cmd_rosbag = ['rosbag', 'record']
            cmd_rosbag.extend(self.rosbag_topics)
            cmd_rosbag.extend(['--output-name', filename])
        cmd_rosbag.extend(self.rosbag_options)
        self.rosbag_process = subprocess.Popen(cmd_rosbag)

    def end_rosbag(self):
        if self.rosbag_process:
            os.kill(self.rosbag_process.pid, signal.SIGTERM)

    def save_topic(self, topic, msg, savetype, filename):
        if savetype == 'ColorImage':
            bridge = cv_bridge.CvBridge()
            img = bridge.imgmsg_to_cv2(msg, 'rgb8')
            dump_ndarray(filename, img)
        elif savetype == 'DepthImage':
            bridge = cv_bridge.CvBridge()
            depth = bridge.imgmsg_to_cv2(msg)
            dump_ndarray(filename, depth)
        elif savetype == 'LabelImage':
            bridge = cv_bridge.CvBridge()
            label = bridge.imgmsg_to_cv2(msg)
            dump_ndarray(filename, label)
        elif savetype == 'YAML':
            if ROS_VERSION == 2:
                msg_yaml = message_to_yaml(msg)
            else:
                msg_yaml = genpy.message.strify_message(msg)
            with open(filename, 'w') as f:
                f.write(msg_yaml)
        else:
            ros_compat.logerr(self.node, 'Unexpected savetype for topic: {}'.format(savetype))
            raise ValueError

    def save_param(self, param, savetype, filename):
        value = ros_compat.get_param(self.node, param) if ROS_VERSION == 1 \
            else self.node.get_parameter(param).value
        if savetype == 'Text':
            with open(filename, 'w') as f:
                f.write(str(value))
        elif savetype == 'YAML':
            content = yaml.safe_dump(value, allow_unicode=True,
                                     default_flow_style=False)
            with open(filename, 'w') as f:
                f.write(content)
        else:
            ros_compat.logerr(self.node, 'Unexpected savetype for param: {}'.format(savetype))
            raise ValueError

    def _sync_save(self):
        stamp = self.msg[self.topics[0]['name']]['stamp']
        stamp_key = str(stamp.to_nsec()) if ROS_VERSION == 1 \
            else str(stamp.sec * 10**9 + stamp.nanosec)
        save_dir = osp.join(self.save_dir, stamp_key)
        if not osp.exists(save_dir):
            os.makedirs(save_dir)
        for topic in self.topics:
            msg = self.msg[topic['name']]['msg']
            filename = osp.join(save_dir, topic['fname'])
            self.save_topic(topic['name'], msg, topic['savetype'], filename)
        for param in self.params:
            filename = osp.join(save_dir, param['fname'])
            self.save_param(param['key'], param['savetype'], filename)
        msg = 'Saved data to {}'.format(save_dir)
        ros_compat.loginfo(self.node, msg)
        return True, msg

    def _save(self):
        now = ros_compat.now_sec(self.node)
        saving_msgs = {}
        while len(saving_msgs) < len(self.topics):
            for topic in self.topics:
                if topic['name'] in saving_msgs:
                    continue
                if topic['name'] not in self.msg:
                    continue
                stamp_sec = ros_compat.stamp_to_sec(self.msg[topic['name']]['stamp'])
                if abs(now - stamp_sec) < self.slop:
                    saving_msgs[topic['name']] = self.msg[topic['name']]['msg']
                if now < stamp_sec:
                    msg = 'timeout for topic [{}]. try bigger slop'.format(topic['name'])
                    ros_compat.logerr(self.node, msg)
                    return False, msg
            # NOTE: under ROS2 this runs inside a service callback that is
            # itself dispatched by the node's executor, so it must not call
            # rclpy.spin_once() here (that would re-enter the executor). A
            # MultiThreadedExecutor (see main()) keeps subscription
            # callbacks running concurrently on other worker threads while
            # this one sleeps.
            ros_compat.sleep(self.node, 0.01)

        if self.timestamp_save_dir:
            now_key = str(int(now * 1e9))
            save_dir = osp.join(self.save_dir, now_key)
        else:
            save_dir = self.save_dir

        if not osp.exists(save_dir):
            os.makedirs(save_dir)
        for topic in self.topics:
            msg = saving_msgs[topic['name']]
            filename = osp.join(save_dir, topic['fname'])
            self.save_topic(topic['name'], msg, topic['savetype'], filename)
        for param in self.params:
            filename = osp.join(save_dir, param['fname'])
            self.save_param(param['key'], param['savetype'], filename)
        msg = 'Saved data to {}'.format(save_dir)
        ros_compat.loginfo(self.node, msg)
        return True, msg

    def start_service_cb(self, req):
        self.start = True
        if self.rosbag_topics:
            self.start_rosbag()
        return True, ''

    def end_service_cb(self, req):
        self.start = False
        if self.rosbag_topics:
            self.end_rosbag()
        return True, ''

    def wait_msgs_update(self):
        now = ros_compat.now_sec(self.node)
        for msg_key in self.msg.keys():
            time_diff = None
            while time_diff is None or time_diff < 0:
                stamp_sec = ros_compat.stamp_to_sec(self.msg[msg_key]['stamp'])
                time_diff = stamp_sec - now
                if ROS_VERSION == 1:
                    rospy.logwarn_throttle(
                        1.0, "msgs is not updated after service request")
                ros_compat.sleep(self.node, 0.05)

    def service_cb(self, req):
        if self.wait_save_request:
            self.wait_msgs_update()
        result, msg = self._save()
        return result, msg

    def sync_service_cb(self, req):
        if self.wait_save_request:
            self.wait_msgs_update()
        result, msg = self._sync_save()
        return result, msg

    def timer_cb(self, event):
        if self.wait_timer:
            self.wait_msgs_update()
        if self.start:
            result, msg = self._save()

    def sync_timer_cb(self, event):
        if self.wait_timer:
            self.wait_msgs_update()
        if self.start:
            result, msg = self._sync_save()


def main():
    if ROS_VERSION == 2:
        rclpy.init()
        node = rclpy.create_node('data_collection_server')
        DataCollectionServer(node)
        # MultiThreadedExecutor so subscription callbacks keep running
        # while a service callback blocks in _save()/wait_msgs_update().
        ros_compat.spin_and_shutdown(node, executor=MultiThreadedExecutor())
    else:
        rospy.init_node('data_collection_server')
        DataCollectionServer()
        rospy.spin()


if __name__ == '__main__':
    main()
