import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    audible_warning_node = launch_ros.actions.Node(
        package='jsk_tools',
        executable='audible_warning.py',
        name='audible_warning',
        output='screen',
        parameters=[{
            'speak_interval': 0.0,
            'seconds_to_start_speaking': 0.0,
        }],
    )
    return launch.LaunchDescription([
        audible_warning_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'audible_warning_node': audible_warning_node}


class TestAudibleWarning(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_audible_warning')

    def tearDown(self):
        self.node.destroy_node()

    def test_speak_on_warn(self):
        pub = self.node.create_publisher(DiagnosticArray, '/diagnostics_agg', 1)
        received = []
        self.node.create_subscription(
            String, '/audible_warning/output/text',
            lambda msg: received.append(msg), 1)

        diag = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'some/leaf/status'
        status.level = DiagnosticStatus.WARN
        status.message = 'a warning message'
        diag.status = [status]

        end_time = time.time() + 10.0
        while time.time() < end_time and not received:
            pub.publish(diag)
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertTrue(received, 'audible_warning did not publish ~output/text')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, audible_warning_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=audible_warning_node,
            allowable_exit_codes=[0, -2, -6, -15])
