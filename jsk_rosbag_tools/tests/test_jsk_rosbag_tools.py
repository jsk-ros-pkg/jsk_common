#!/usr/bin/env python

import os.path as osp
import subprocess
import unittest

import rospkg


PKG = 'jsk_rosbag_tools'
NAME = 'test_jsk_rosbag_tools'


class TestJSKRosBagTools(unittest.TestCase):

    def test_bag_to_audio(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools bag_to_audio.py {}'.format(
            video_bag_path)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

    def test_tf_static_to_tf(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools tf_static_to_tf.py {} ' \
            '--no-progress-bar'.format(video_bag_path)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

    def test_merge(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')

        output_path = osp.join(path, 'tests', 'output', 'merged.bag')
        audio_bag_path = osp.join(path, 'samples', 'data',
                                  '2022-05-07-hello-test.bag')
        cmd = 'rosrun jsk_rosbag_tools merge.py {} {} -o {}'\
            .format(audio_bag_path, audio_bag_path, output_path)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

    def test_bag_to_video_and_video_to_bag_and_compress(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('jsk_rosbag_tools')

        output_dir = osp.join(path, 'tests', 'output', 'audio')
        audio_bag_path = osp.join(path, 'samples', 'data',
                                  '2022-05-07-hello-test.bag')

        cmd = 'rosrun jsk_rosbag_tools bag_to_video.py {} -o {}'.format(
            audio_bag_path, output_dir)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

        output_dir = osp.join(path, 'tests', 'output', 'video')
        video_bag_path = osp.join(path, 'samples', 'data',
                                  '20220530173950_go_to_kitchen_rosbag.bag')
        cmd = 'rosrun jsk_rosbag_tools bag_to_video.py {} -o {}'.format(
            video_bag_path, output_dir)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

        # video_to_bag.py test
        video_path = osp.join(
            output_dir,
            'head_camera--slash--rgb--slash--throttled'
            '--slash--image_rect_color--slash--compressed.mp4')
        output_filename = osp.join(path, 'tests', 'output', 'video_to_bag.bag')
        cmd = 'rosrun jsk_rosbag_tools video_to_bag.py {} ' \
            '--out {} --no-progress-bar'.format(
                video_path, output_filename)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError

        # compress_imgs.py test
        cmd = 'rosrun jsk_rosbag_tools compress_imgs.py {} ' \
            '--no-progress-bar'.format(output_filename)
        proc = subprocess.Popen(cmd, shell=True)
        proc.wait()

        if proc.returncode != 0:
            raise RuntimeError


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestJSKRosBagTools)
