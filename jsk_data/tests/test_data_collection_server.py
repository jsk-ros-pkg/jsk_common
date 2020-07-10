#!/usr/bin/env python

from __future__ import print_function

import os
import os.path as osp
import shutil
import unittest

import numpy as np
import scipy.misc
from distutils.version import StrictVersion
import scipy.version
if (StrictVersion(scipy.version.version) > StrictVersion('1.2.0')):
    import imageio
import yaml

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


PKG = 'jsk_data'
NAME = 'test_data_collection_server'


class TestDataCollectionServer(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def check(self, save_dir, target='string'):
        sub_dirs = os.listdir(save_dir)
        self.assertGreater(len(sub_dirs), 0)

        rospy.sleep(1)

        for sub_dir in sub_dirs:
            sub_dir = osp.join(save_dir, sub_dir)
            with open(osp.join(sub_dir, 'sample_string.txt')) as f:
                self.assertEqual(f.read(), 'spam')
            if target == 'string':
                with open(osp.join(sub_dir, 'sample_topic.yaml')) as f:
                    data = yaml.load(f)
                    self.assertEqual(data['data'], 'sample')
            elif target == 'image':
                if (StrictVersion(scipy.version.version) > StrictVersion('1.2.0')):
                    img = imageio.imread(osp.join(sub_dir, 'sample_image.png'))
                else:
                    img = scipy.misc.imread(osp.join(sub_dir, 'sample_image.png'))
                self.assertTrue(np.allclose(img, scipy.misc.face()))
            else:
                raise ValueError('Unexpected target: {}'.format(target))

    def test_request(self):
        rospy.wait_for_message('/sample_topic', String)
        rospy.wait_for_service('/data_collection_server_request/save_request')
        rospy.sleep(2)
        save_request = rospy.ServiceProxy(
            '/data_collection_server_request/save_request', Trigger
        )
        ret = save_request()
        self.assertTrue(ret.success)

        save_dir = rospy.get_param('/save_dir_request')
        save_dir = osp.expanduser(save_dir)
        self.check(save_dir, target='string')

    def test_timer(self):
        rospy.wait_for_message('/sample_topic', String)
        rospy.wait_for_service('/data_collection_server_timer/start_request')
        start_request = rospy.ServiceProxy(
            '/data_collection_server_timer/start_request', Trigger
        )
        ret = start_request()
        rospy.sleep(2)
        self.assertTrue(ret.success)

        save_dir = rospy.get_param('/save_dir_timer')
        save_dir = osp.expanduser(save_dir)
        self.check(save_dir, target='string')

    def test_all(self):
        rospy.wait_for_message('/static_image_publisher/output', Image)
        rospy.sleep(2)

        save_dir = rospy.get_param('/save_dir_all')
        save_dir = osp.expanduser(save_dir)
        self.check(save_dir, target='image')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDataCollectionServer)
