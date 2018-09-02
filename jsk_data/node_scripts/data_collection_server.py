#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import os.path as osp
import pickle as pkl
import sys

import numpy as np
import PIL.Image
import yaml

import cv_bridge
import dynamic_reconfigure.server
import genpy
from jsk_topic_tools.log_utils import jsk_logfatal
import message_filters
import roslib.message
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from jsk_data.cfg import DataCollectionServerConfig


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

    def __init__(self):
        dynamic_reconfigure.server.Server(
            DataCollectionServerConfig, self.reconfig_cb)
        self.msg = {}
        self.topics = rospy.get_param('~topics', [])
        # validation for saving topics
        for topic in self.topics:
            required_fields = ['name', 'msg_class', 'fname', 'savetype']
            for field in required_fields:
                if field not in topic:
                    jsk_logfatal("Required field '{}' for topic is missing"
                                 .format(field))
                    sys.exit(1)
        self.params = rospy.get_param('~params', [])
        self.slop = rospy.get_param('~slop', 0.1)
        # validation for saving params
        for param in self.params:
            required_fields = ['key', 'fname', 'savetype']
            for field in required_fields:
                if field not in param:
                    jsk_logfatal("Required field '{}' for param is missing"
                                 .format(field))
                    sys.exit(1)

        method = rospy.get_param('~method', 'request')
        if method not in ['request', 'timer', 'all', 'message_filters']:
            raise ValueError('Unexpected method: {}'.format(method))
        use_message_filters = rospy.get_param('~message_filters', False)
        self.timestamp_save_dir = rospy.get_param('~timestamp_save_dir', True)

        if rospy.has_param('~with_request'):
            rospy.logwarn('Deprecated param: ~with_request, Use ~method')
            if not rospy.get_param('~with_request'):
                use_message_filters = True
                method = 'all'
        if method == 'message_filters':
            rospy.logwarn(
                'Deprecated param: ~method: message_filters,'
                'Use ~message_filters: true')
            use_message_filters = True
            method = 'all'

        # set subscribers
        self.subs = []
        for topic in self.topics:
            msg_class = roslib.message.get_message_class(
                topic['msg_class'])
            if use_message_filters:
                sub = message_filters.Subscriber(
                    topic['name'], msg_class)
            else:
                sub = rospy.Subscriber(
                    topic['name'], msg_class, self.sub_cb,
                    callback_args=topic['name'])
            self.subs.append(sub)

        # add synchoronizer if use_message_filters
        if use_message_filters:
            queue_size = rospy.get_param('~queue_size', 10)
            approximate_sync = rospy.get_param('~approximate_sync', False)
            if approximate_sync:
                slop = rospy.get_param('~slop', 0.1)
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    self.subs, queue_size=queue_size, slop=slop)
            else:
                self.sync = message_filters.TimeSynchronizer(
                    self.subs, queue_size=queue_size)

        # set collecting method
        if method == 'request':
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_cb)
                self.server = rospy.Service(
                    '~save_request', Trigger, self.sync_service_cb)
            else:
                self.server = rospy.Service(
                    '~save_request', Trigger, self.service_cb)
        elif method == 'timer':
            duration = rospy.Duration(1.0 / rospy.get_param('~hz', 1.0))
            self.start = False
            self.start_server = rospy.Service(
                '~start_request', Trigger, self.start_service_cb)
            self.end_server = rospy.Service(
                '~end_request', Trigger, self.end_service_cb)
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_cb)
                self.timer = rospy.Timer(duration, self.sync_timer_cb)
            else:
                self.timer = rospy.Timer(duration, self.timer_cb)
        else:
            assert method == 'all'
            if use_message_filters:
                self.sync.registerCallback(self.sync_sub_and_save_cb)
            else:
                rospy.logerr(
                    '~use_filters: False, ~method: all is not supported')
                sys.exit(1)

    def reconfig_cb(self, config, level):
        self.save_dir = osp.expanduser(config['save_dir'])
        if not osp.exists(self.save_dir):
            os.makedirs(self.save_dir)
        return config

    def __del__(self):
        for sub in self.subs:
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
        self.msg[topic_name] = {
            'stamp': msg.header.stamp if msg._has_header else rospy.Time.now(),
            'msg': msg
        }

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
            msg_yaml = genpy.message.strify_message(msg)
            with open(filename, 'w') as f:
                f.write(msg_yaml)
        else:
            rospy.logerr('Unexpected savetype for topic: {}'.format(savetype))
            raise ValueError

    def save_param(self, param, savetype, filename):
        value = rospy.get_param(param)
        if savetype == 'Text':
            with open(filename, 'w') as f:
                f.write(str(value))
        elif savetype == 'YAML':
            content = yaml.safe_dump(value, allow_unicode=True,
                                     default_flow_style=False)
            with open(filename, 'w') as f:
                f.write(content)
        else:
            rospy.logerr('Unexpected savetype for param: {}'.format(savetype))
            raise ValueError

    def _sync_save(self):
        stamp = self.msg[self.topics[0]['name']]['stamp']
        save_dir = osp.join(self.save_dir, str(stamp.to_nsec()))
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
        rospy.loginfo(msg)
        return True, msg

    def _save(self):
        now = rospy.Time.now()
        saving_msgs = {}
        while len(saving_msgs) < len(self.topics):
            for topic in self.topics:
                if topic['name'] in saving_msgs:
                    continue
                if topic['name'] not in self.msg:
                    continue
                stamp = self.msg[topic['name']]['stamp']
                if abs(now - stamp) < rospy.Duration(self.slop):
                    saving_msgs[topic['name']] = self.msg[topic['name']]['msg']
                if now < stamp:
                    msg = 'timeout for topic [{}]. try bigger slop'.format(
                        topic['name'])
                    rospy.logerr(msg)
                    return False, msg
            rospy.sleep(0.01)

        if self.timestamp_save_dir:
            save_dir = osp.join(self.save_dir, str(now.to_nsec()))
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
        rospy.loginfo(msg)
        return True, msg

    def start_service_cb(self, req):
        self.start = True
        return TriggerResponse(success=True)

    def end_service_cb(self, req):
        self.start = False
        return TriggerResponse(success=True)

    def service_cb(self, req):
        result, msg = self._save()
        if result:
            return TriggerResponse(success=True, message=msg)
        else:
            return TriggerResponse(success=False, message=msg)

    def sync_service_cb(self, req):
        result, msg = self._sync_save()
        if result:
            return TriggerResponse(success=True, message=msg)
        else:
            return TriggerResponse(success=False, message=msg)

    def timer_cb(self, event):
        if self.start:
            result, msg = self._save()

    def sync_timer_cb(self, event):
        if self.start:
            result, msg = self._sync_save()


if __name__ == '__main__':
    rospy.init_node('data_collection_server')
    server = DataCollectionServer()
    rospy.spin()
