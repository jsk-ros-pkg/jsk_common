#!/usr/bin/env python

import os.path as osp
import subprocess
import unittest

import rospkg
import rospy


PKG = 'jsk_rosbag_tools'
NAME = 'test_bag_to_video'


class TestBagToVideo(unittest.TestCase):

    def _check_command(self, cmd):
        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE)

        lines = ''
        with proc.stdout:
            for line in iter(proc.stdout.readline, b''):
                lines += line.decode('utf-8').strip()
        returncode = proc.wait()

        if returncode != 0:
            rospy.logerr('{}'.format(lines))
            rospy.logerr('command {} failed.'.format(cmd))
            raise RuntimeError('command {} failed.'.format(cmd))

    def test_bag_to_video_and_video_to_bag_and_compress(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')

        output_dir = osp.join(path, 'tests', 'output', 'audio')
        audio_bag_path = osp.join(path, 'samples', 'data',
                                  '2022-05-07-hello-test.bag')

        cmd = 'rosrun jsk_rosbag_tools bag_to_video.py {} -o {}'.format(
            audio_bag_path, output_dir)
        self._check_command(cmd)

        output_dir = osp.join(path, 'tests', 'output', 'video')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools bag_to_video.py {} -o {}'.format(
            video_bag_path, output_dir)
        self._check_command(cmd)

        # video_to_bag.py test
        video_path = osp.join(
            output_dir,
            'head_camera--slash--rgb--slash--throttled'
            '--slash--image_rect_color--slash--compressed.mp4')
        output_filename = osp.join(path, 'tests', 'output', 'video_to_bag.bag')
        cmd = 'rosrun jsk_rosbag_tools video_to_bag.py {} ' \
            '--out {} --no-progress-bar'.format(
                video_path, output_filename)
        self._check_command(cmd)

        # compress_imgs.py test
        cmd = 'rosrun jsk_rosbag_tools compress_imgs.py {} ' \
            '--no-progress-bar'.format(output_filename)
        self._check_command(cmd)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestBagToVideo)
