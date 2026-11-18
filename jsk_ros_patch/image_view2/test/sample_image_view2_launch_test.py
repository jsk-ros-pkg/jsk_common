import os
import time
import unittest

import launch
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_ros.actions import PushRosNamespace
import pytest

import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


SAMPLE_DIR = os.path.join(os.path.dirname(__file__), '..', 'sample')
# not to receive the images of the other launch tests running at the same time
NAMESPACE = 'sample_image_view2_test'


@pytest.mark.launch_test
def generate_test_description():
    sample = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(SAMPLE_DIR, 'sample_image_view2.launch.xml')),
        launch_arguments={
            'gui': 'false',
            'file_name': os.path.join(SAMPLE_DIR,
                                      'kiva_pod_image_color.jpg'),
        }.items(),
    )
    return launch.LaunchDescription([
        launch.actions.GroupAction([PushRosNamespace(NAMESPACE), sample]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSampleImageView2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_sample_image_view2')

    def tearDown(self):
        self.node.destroy_node()

    def test_marked(self):
        marked = []
        self.node.create_subscription(
            Image, '/%s/camera/image/marked' % NAMESPACE,
            lambda msg: marked.append(msg), 1)
        bridge = CvBridge()

        # the outline of the shelf (a green LINE_STRIP of
        # publish_sample_image.py from (110, 175) to (110, 355)) is drawn
        green = None
        end_time = time.time() + 30.0
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if not marked:
                continue
            img = bridge.imgmsg_to_cv2(marked[-1], 'bgr8')
            self.assertEqual(img.shape, (360, 640, 3))
            green = tuple(int(v) for v in img[265, 110])
            if green == (0, 255, 0):
                break
        self.assertTrue(marked, 'no /camera/image/marked')
        self.assertEqual(green, (0, 255, 0),
                         'the shelf outline is not drawn on marked')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -6, -15])
