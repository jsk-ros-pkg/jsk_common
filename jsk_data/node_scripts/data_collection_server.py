#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import os.path as osp
import pickle as pkl
import sys

import cv2
import yaml

import cv_bridge
import dynamic_reconfigure.server
from jsk_topic_tools.log_utils import jsk_logfatal
import roslib.message
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from jsk_data.cfg import DataCollectionServerConfig


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
        # validation for saving params
        for param in self.params:
            required_fields = ['key', 'fname', 'savetype']
            for field in required_fields:
                if field not in topic:
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
        self.msg[topic_name] = msg

    def service_cb(self, req):
        now = rospy.Time.now()
        saving_msgs = {}
        slop = 0.1
        while len(saving_msgs) < len(self.topics):
            for topic in self.topics:
                if topic['name'] in saving_msgs:
                    continue
                if ((topic['name'] in self.msg) and
                     abs(now - self.msg[topic['name']].header.stamp <
                         rospy.Duration(slop))):
                    saving_msgs[topic['name']] = self.msg[topic['name']]
            rospy.sleep(0.1)
        save_dir = osp.join(self.save_dir, str(now.to_nsec()))
        if not osp.exists(save_dir):
            os.makedirs(save_dir)
        for topic in self.topics:
            msg = saving_msgs[topic['name']]
            if topic['savetype'] == 'ColorImage':
                bridge = cv_bridge.CvBridge()
                img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(osp.join(save_dir, topic['fname']), img)
            elif topic['savetype'] == 'DepthImage':
                bridge = cv_bridge.CvBridge()
                depth = bridge.imgmsg_to_cv2(msg)
                with open(osp.join(save_dir, topic['fname']), 'wb') as f:
                    pkl.dump(depth, f)
            elif topic['savetype'] == 'LabelImage':
                bridge = cv_bridge.CvBridge()
                label = bridge.imgmsg_to_cv2(msg)
                cv2.imwrite(osp.join(save_dir, topic['fname']), label)
            else:
                rospy.logerr('Unexpected savetype for topic: {}'
                             .format(topic['savetype']))
                raise ValueError
        for param in self.params:
            value = rospy.get_param(param['key'])
            if param['savetype'] == 'Text':
                with open(osp.join(save_dir, param['fname']), 'w') as f:
                    f.write(str(value))
            elif param['savetype'] == 'YAML':
                content = yaml.safe_dump(value, allow_unicode=True,
                                         default_flow_style=False)
                with open(osp.join(save_dir, param['fname']), 'w') as f:
                    f.write(content)
            else:
                rospy.logerr('Unexpected savetype for param: {}'
                             .format(param['savetype']))
                raise ValueError
        message = 'Saved data to {}'.format(save_dir)
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)


if __name__ == '__main__':
    rospy.init_node('data_collection_server')
    server = DataCollectionServer()
    rospy.spin()
