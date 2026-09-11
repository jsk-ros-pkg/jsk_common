import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

import rclpy
from tf2_ros import Buffer, TransformListener
from dynamic_tf_publisher.srv import SetDynamicTF


@pytest.mark.launch_test
def generate_test_description():
    tf_publish_node = launch_ros.actions.Node(
        package='dynamic_tf_publisher',
        executable='tf_publish.py',
        name='tf_publish_server',
        output='screen',
    )
    return launch.LaunchDescription([
        tf_publish_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'tf_publish_node': tf_publish_node}


class TestTfPublish(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_tf_publish')

    def tearDown(self):
        self.node.destroy_node()

    def test_set_dynamic_tf_and_lookup(self):
        client = self.node.create_client(SetDynamicTF, '/set_dynamic_tf')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        req = SetDynamicTF.Request()
        req.freq = 10.0
        req.cur_tf.header.frame_id = 'map'
        req.cur_tf.child_frame_id = 'test_child'
        req.cur_tf.transform.rotation.w = 1.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())

        buffer = Buffer()
        listener = TransformListener(buffer, self.node)

        found = False
        end_time = time.time() + 10.0
        while time.time() < end_time and not found:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            found = buffer.can_transform('map', 'test_child', rclpy.time.Time())
        self.assertTrue(found, 'test_child frame was not broadcast within timeout')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, tf_publish_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=tf_publish_node,
            allowable_exit_codes=[0, -2, -6, -15])
