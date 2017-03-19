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
        self.server = rospy.Service('~save_request', Trigger, self.service_cb)
        self.subs = []
        for topic in self.topics:
            msg_class = roslib.message.get_message_class(topic['msg_class'])
            sub = rospy.Subscriber(topic['name'], msg_class, self.sub_cb,
                                   callback_args=topic['name'])
            self.subs.append(sub)

    def reconfig_cb(self, config, level):
        self.save_dir = osp.expanduser(config['save_dir'])
        if not osp.exists(self.save_dir):
            os.makedirs(self.save_dir)
        return config

    def __del__(self):
        for sub in self.subs:
            sub.unregister()

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

    def service_cb(self, req):
        now = rospy.Time.now()
        saving_msgs = {}
        while len(saving_msgs) < len(self.topics):
            for topic in self.topics:
                if topic['name'] in saving_msgs:
                    continue
                stamp = self.msg[topic['name']]['stamp']
                if ((topic['name'] in self.msg) and
                        abs(now - stamp) < rospy.Duration(self.slop)):
                    saving_msgs[topic['name']] = self.msg[topic['name']]['msg']
                if now < stamp:
                    rospy.logwarn(
                        'timestamp exceeds starting time, try bigger slop')
            rospy.sleep(self.slop)
        save_dir = osp.join(self.save_dir, str(now.to_nsec()))
        if not osp.exists(save_dir):
            os.makedirs(save_dir)
        for topic in self.topics:
            msg = saving_msgs[topic['name']]
            filename = osp.join(save_dir, topic['fname'])
            self.save_topic(topic['name'], msg, topic['savetype'], filename)
        for param in self.params:
            filename = osp.join(save_dir, param['fname'])
            self.save_param(param['key'], param['savetype'], filename)
        message = 'Saved data to {}'.format(save_dir)
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)


if __name__ == '__main__':
    rospy.init_node('data_collection_server')
    server = DataCollectionServer()
    rospy.spin()
