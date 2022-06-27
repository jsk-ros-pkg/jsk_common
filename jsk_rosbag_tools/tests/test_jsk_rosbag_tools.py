#!/usr/bin/env python

import os.path as osp
import subprocess
import unittest

import rospkg
import rospy


PKG = 'jsk_rosbag_tools'
NAME = 'test_jsk_rosbag_tools'


class TestJSKRosBagTools(unittest.TestCase):

    def _check_command(self, cmd):
        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE)
        with proc.stdout:
            for line in iter(proc.stdout.readline, b''):
                rospy.loginfo('{}'.format(line.decode('utf-8').strip()))
        returncode = proc.wait()

        if returncode != 0:
            raise RuntimeError('command {} failed.'.format(cmd))

    def test_bag_to_audio(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools bag_to_audio.py {}'.format(
            video_bag_path)
        self._check_command(cmd)

    def test_tf_static_to_tf(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools tf_static_to_tf.py {} ' \
            '--no-progress-bar'.format(video_bag_path)
        self._check_command(cmd)

    def test_merge(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')

        output_path = osp.join(path, 'tests', 'output', 'merged.bag')
        audio_bag_path = osp.join(path, 'samples', 'data',
                                  '2022-05-07-hello-test.bag')
        cmd = 'rosrun jsk_rosbag_tools merge.py {} {} -o {}'\
            .format(audio_bag_path, audio_bag_path, output_path)
        self._check_command(cmd)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestJSKRosBagTools)
