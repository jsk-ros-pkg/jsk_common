import os
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    save_dir = tempfile.mkdtemp(prefix='jsk_data_test_')
    data_collection_server_node = launch_ros.actions.Node(
        package='jsk_data',
        executable='data_collection_server.py',
        name='data_collection_server',
        output='screen',
        parameters=[{
            'save_dir': save_dir,
            'method': 'request',
            'timestamp_save_dir': False,
            'topics_name': ['/test_topic'],
            'topics_msg_class': ['std_msgs/msg/String'],
            'topics_fname': ['data.yaml'],
            'topics_savetype': ['YAML'],
        }],
    )
    return launch.LaunchDescription([
        data_collection_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'data_collection_server_node': data_collection_server_node,
        'save_dir': save_dir,
    }


class TestDataCollectionServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_data_collection_server')

    def tearDown(self):
        self.node.destroy_node()

    def test_save_request(self, save_dir):
        pub = self.node.create_publisher(String, '/test_topic', 1)

        client = self.node.create_client(
            Trigger, '/data_collection_server/save_request')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        end_time = time.time() + 10.0
        while time.time() < end_time:
            pub.publish(String(data='hello'))
            rclpy.spin_once(self.node, timeout_sec=0.1)

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        response = future.result()
        self.assertTrue(response.success, response.message)
        self.assertTrue(os.path.exists(os.path.join(save_dir, 'data.yaml')))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, data_collection_server_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=data_collection_server_node,
            allowable_exit_codes=[0, -2, -6, -15])
