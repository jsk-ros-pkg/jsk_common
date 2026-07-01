#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2026, JSK Lab
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the JSK Lab nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

"""
test_connection_based_lifecycle.py
Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>

launch_testing test for ConnectionBasedLifecycleNode.
Verifies that a node inheriting ConnectionBasedLifecycleNode:
  - subscribes to input when a subscriber connects to its output
  - unsubscribes from input when no subscriber remains on its output

Ported from the ROS1 test_connection_based_nodelet.test approach.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import pytest

import rclpy
from std_msgs.msg import String


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    string_relay_lifecycle_node = launch_ros.actions.Node(
        package='jsk_topic_tools',
        executable='string_relay_lifecycle_node',
        name='string_relay_lifecycle',
        output='screen',
    )
    return launch.LaunchDescription([
        string_relay_lifecycle_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestConnectionBasedLifecycleNode(unittest.TestCase):

    LIFECYCLE_NODE_NAME = 'string_relay_lifecycle'
    INPUT_TOPIC = '/string_relay_lifecycle/input'
    OUTPUT_TOPIC = '/string_relay_lifecycle/output'

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_connection_based_lifecycle')

    def tearDown(self):
        self.node.destroy_node()

    # -- helpers --

    def _get_lifecycle_subs_on_input(self):
        """Return subscription info for the lifecycle node on the input topic."""
        subs_info = self.node.get_subscriptions_info_by_topic(
            self.INPUT_TOPIC)
        return [
            s for s in subs_info
            if s.node_name == self.LIFECYCLE_NODE_NAME
        ]

    def _wait_for_subscription(self, present, timeout_sec=10.0):
        """Poll until the lifecycle node's subscription on input
        appears (present=True) or disappears (present=False)."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            subs = self._get_lifecycle_subs_on_input()
            if present and len(subs) > 0:
                return True
            if not present and len(subs) == 0:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.5)
        return False

    # -- tests --

    def test_connection_based_lifecycle(self):
        """Verify connection-based subscribe/unsubscribe behavior."""

        # 1. Subscribe to the output topic -> the node should subscribe to input
        sub = self.node.create_subscription(
            String, self.OUTPUT_TOPIC,
            lambda msg: None, 10)

        self.assertTrue(
            self._wait_for_subscription(present=True, timeout_sec=10.0),
            'Node should subscribe to input when output has subscribers')

        # 2. Unsubscribe from output -> the node should unsubscribe from input
        self.node.destroy_subscription(sub)

        self.assertTrue(
            self._wait_for_subscription(present=False, timeout_sec=10.0),
            'Node should unsubscribe from input after output subscribers leave')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15])
