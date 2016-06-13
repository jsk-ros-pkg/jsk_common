#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import os.path as osp
import pickle as pkl

import cv2

import cv_bridge
import roslib.message
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


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
        self.msg = {}
        self.save_dir = osp.expanduser(rospy.get_param('~save_dir', '~/.ros'))
        if not osp.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.topics = rospy.get_param('~topics')
        self.params = rospy.get_param('~params')
        self.server = rospy.Service('~save_request', Trigger, self.service_cb)
        self.subs = []
        for topic in self.topics:
            msg_class = roslib.message.get_message_class(topic['msg_class'])
            sub = rospy.Subscriber(topic['name'], msg_class, self.sub_cb,
                                   callback_args=topic['name'])
            self.subs.append(sub)

    def __del__(self):
        for sub in self.subs:
            sub.unregister()

    def sub_cb(self, msg, topic_name):
        self.msg[topic_name] = msg

    def service_cb(self, req):
        now = rospy.Time.now()
        while any(msg.header.stamp < now for msg in self.msg.values()):
            rospy.sleep(0.1)
        save_dir = osp.join(self.save_dir, str(now.to_nsec()))
        if not osp.exists(save_dir):
            os.makedirs(save_dir)
        for topic in self.topics:
            msg = self.msg.get(topic['name'], None)
            if msg is None:
                os.removedirs(save_dir)
                return TriggerResponse(success=False)
            if topic['savetype'] == 'ColorImage':
                bridge = cv_bridge.CvBridge()
                img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(osp.join(save_dir, topic['fname']), img)
            elif topic['savetype'] == 'DepthImage':
                bridge = cv_bridge.CvBridge()
                depth = bridge.imgmsg_to_cv2(msg)
                with open(osp.join(save_dir, topic['fname']), 'wb') as f:
                    pkl.dump(depth, f)
            else:
                raise ValueError
        for param in self.params:
            value = rospy.get_param(param['key'])
            if param['savetype'] == 'Text':
                with open(osp.join(save_dir, param['fname']), 'w') as f:
                    f.write(str(value))
            else:
                raise ValueError
        rospy.loginfo('Saved data to {}'.format(save_dir))
        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('data_collection_server')
    server = DataCollectionServer()
    rospy.spin()
