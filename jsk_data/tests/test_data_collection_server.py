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

    def _wait_until_data_saved(self, target_dir, required_file_num=1):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(os.listdir(target_dir)) > 0:
                for sub_dir in os.listdir(target_dir):
                    if len(os.listdir(osp.join(target_dir, sub_dir))) \
                            >= required_file_num:
                        return
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue

    def check(self, save_dir, target='string'):
        sub_dirs = os.listdir(save_dir)
        self.assertGreater(len(sub_dirs), 0)

        rospy.sleep(1)

        successfully_saved_count = 0
        for sub_dir in sub_dirs:
            sub_dir = osp.join(save_dir, sub_dir)
            text_filepath = osp.join(sub_dir, 'sample_string.txt')

            # Check if text and yaml or text and image are
            # saved in the subdirectory of `save_dir`.
            if target == 'string':
                yaml_filepath = osp.join(sub_dir, 'sample_topic.yaml')
                if not (osp.exists(text_filepath)
                        and osp.exists(yaml_filepath)):
                    continue
            else:
                img_filepath = osp.join(sub_dir, 'sample_image.png')
                if not (osp.exists(text_filepath)
                        and osp.exists(img_filepath)):
                    continue

            with open(text_filepath) as f:
                self.assertEqual(f.read(), 'spam')
            if target == 'string':
                with open(yaml_filepath) as f:
                    data = yaml.load(f, Loader=yaml.SafeLoader)
                    self.assertEqual(data['data'], 'sample')
            elif target == 'image':
                if (StrictVersion(scipy.version.version) > StrictVersion('1.2.0')):
                    img = imageio.imread(img_filepath)
                else:
                    img = scipy.misc.imread(img_filepath)
                self.assertTrue(np.allclose(img, scipy.misc.face()))
            else:
                raise ValueError('Unexpected target: {}'.format(target))
            successfully_saved_count += 1
        if successfully_saved_count == 0:
            rospy.logerr('Data are not successfully saved in {}'
                         .format(save_dir))

    def test_request(self):
        rospy.wait_for_message('/sample_topic', String)
        rospy.wait_for_service('/data_collection_server_request/save_request')
        save_request = rospy.ServiceProxy(
            '/data_collection_server_request/save_request', Trigger
        )
        ret = save_request()
        self.assertTrue(ret.success)

        save_dir = rospy.get_param('/save_dir_request')
        save_dir = save_dir.rstrip()
        save_dir = osp.expanduser(save_dir)
        self._wait_until_data_saved(save_dir, required_file_num=2)
        self.check(save_dir, target='string')

    def test_timer(self):
        rospy.wait_for_message('/sample_topic', String)
        rospy.wait_for_service('/data_collection_server_timer/start_request')
        start_request = rospy.ServiceProxy(
            '/data_collection_server_timer/start_request', Trigger
        )
        ret = start_request()
        self.assertTrue(ret.success)

        save_dir = rospy.get_param('/save_dir_timer')
        save_dir = save_dir.rstrip()
        save_dir = osp.expanduser(save_dir)
        self._wait_until_data_saved(save_dir, required_file_num=2)
        self.check(save_dir, target='string')

    def test_all(self):
        rospy.wait_for_message('/static_image_publisher/output', Image)

        save_dir = rospy.get_param('/save_dir_all')
        save_dir = save_dir.rstrip()
        save_dir = osp.expanduser(save_dir)
        self._wait_until_data_saved(save_dir, required_file_num=2)
        self.check(save_dir, target='image')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDataCollectionServer)
