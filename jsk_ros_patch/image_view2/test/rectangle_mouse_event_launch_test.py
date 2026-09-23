import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from image_view2.msg import MouseEvent


@pytest.mark.launch_test
def generate_test_description():
    image_view2_node = launch_ros.actions.Node(
        package='image_view2',
        executable='image_view2',
        name='image_view2',
        output='screen',
        remappings=[('image', 'camera/image')],
        parameters=[{'use_window': False}],
    )
    return launch.LaunchDescription([
        image_view2_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'image_view2_node': image_view2_node}


class TestImageView2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_image_view2')

    def tearDown(self):
        self.node.destroy_node()

    def test_rectangle_mouse_event(self):
        image_pub = self.node.create_publisher(Image, '/camera/image', 1)
        event_pub = self.node.create_publisher(
            MouseEvent, '/camera/image/event', 1)

        received = []
        self.node.create_subscription(
            Image, '/camera/image/screenrectangle_image',
            lambda msg: received.append(msg), 1)

        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(
            np.zeros((240, 320, 3), dtype=np.uint8), encoding='bgr8')

        def publish_image():
            img_msg.header.stamp = self.node.get_clock().now().to_msg()
            image_pub.publish(img_msg)

        # let image_view2 pick up at least one image first
        end_time = time.time() + 10.0
        while time.time() < end_time:
            publish_image()
            rclpy.spin_once(self.node, timeout_sec=0.2)

        def send_event(event_type, x, y):
            msg = MouseEvent()
            msg.type = event_type
            msg.x = x
            msg.y = y
            msg.width = 320
            msg.height = 240
            event_pub.publish(msg)

        end_time = time.time() + 10.0
        while time.time() < end_time and not received:
            publish_image()
            send_event(MouseEvent.MOUSE_LEFT_DOWN, 50, 50)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            send_event(MouseEvent.MOUSE_MOVE, 150, 150)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            send_event(MouseEvent.MOUSE_LEFT_UP, 150, 150)
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.assertTrue(
            received, 'image_view2 did not publish screenrectangle_image')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, image_view2_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=image_view2_node,
            allowable_exit_codes=[0, -2, -6, -15])
