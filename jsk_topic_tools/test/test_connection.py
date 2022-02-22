#!/usr/bin/env python

import os
import sys

import unittest

import rosgraph
import rospy
import rosmsg
import roslib


PKG = 'jsk_topic_tools'
NAME = 'test_connection'


class TestConnection(unittest.TestCase):

    def __init__(self, *args):
        super(TestConnection, self).__init__(*args)
        rospy.init_node(NAME)

    def test_no_subscribers(self):
        check_connected_topics = rospy.get_param('~check_connected_topics')
        master = rosgraph.Master('/test_connection')
        _, sub, _ = master.getSystemState()
        # Check assumed topics are not there
        master = rosgraph.Master('test_connection')
        _, subscriptions, _ = master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    raise ValueError('Found topic: {}'.format(check_topic))

    def test_subscriber_appears_disappears(self):
        topic_type = rospy.get_param('~input_topic_type')
        check_connected_topics = rospy.get_param('~check_connected_topics')
        wait_time = rospy.get_param('~wait_for_connection', 0)
        msg_class = roslib.message.get_message_class(topic_type)
        # Subscribe topic and bond connection
        sub = rospy.Subscriber('~input', msg_class,
                               self._cb_test_subscriber_appears_disappears)
        print('Waiting for connection for {} sec.'.format(wait_time))
        rospy.sleep(wait_time)
        # Check assumed topics are there
        master = rosgraph.Master('test_connection')
        _, subscriptions, _ = master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    break
            else:
                raise ValueError('Not found topic: {}'.format(check_topic))
        sub.unregister()
        # FIXME: below test won't pass on hydro
        ROS_DISTRO = os.environ.get('ROS_DISTRO', 'indigo')
        if ord(ROS_DISTRO[0]) < ord('i'):
            sys.stderr.write('WARNING: running on rosdistro %s, and skipping '
                             'test for disconnection.\n' % ROS_DISTRO)
            return
        rospy.sleep(1)  # wait for disconnection
        # Check specified topics do not exist
        _, subscriptions, _ = master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    raise ValueError('Found topic: {}'.format(check_topic))

    def _cb_test_subscriber_appears_disappears(self, msg):
        pass

    def _cb_diagnostic(self, msg):
        self.diagnostic_msg = msg

    def get_diagnostic_message(self):
        self.diagnostic_msg = None
        sub_diagnostic = rospy.Subscriber(
            '/diagnostics',
            DiagnosticArray, self._cb_diagnostic)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.diagnostic_msg is None:
            rate.sleep()
        sub_diagnostic.unregister()
        return self.diagnostic_msg

    def _check_diagnostic_error_level(self, target_hardware_id, error_level):
        while not rospy.is_shutdown():
            msg = self.get_diagnostic_message()
            if len(msg.status) > 0:
                for status in msg.status:
                    if status.hardware_id != target_hardware_id:
                        continue
                    if status.level != error_level:
                        raise ValueError(
                            'ERROR Level is strange: expected {}, but {}'
                            .format(error_level, status.level))
                    return True

    def test_no_subscribers_diagnostic(self):
        target_hardware_id = rospy.get_param('~hardware_id')
        rospy.wait_for_service('/mux/select')
        mux_selector = rospy.ServiceProxy('/mux/select', MuxSelect)

        mux_selector('input')

        # no subsrcibers and error level == DiagnosticStatus.OK
        self._check_diagnostic_error_level(
            target_hardware_id, DiagnosticStatus.OK)

        # start subscribe
        topic_type = rospy.get_param('~input_topic_type')
        msg_class = roslib.message.get_message_class(topic_type)
        # Subscribe topic and bond connection
        sub = rospy.Subscriber('~input', msg_class,
                               self._cb_test_subscriber_appears_disappears)

        # subsrciber exists and error level == DiagnosticStatus.OK
        self._check_diagnostic_error_level(
            target_hardware_id, DiagnosticStatus.OK)

        mux_selector('input_dummy')

        # subscriber exists and error level == 2
        self._check_diagnostic_error_level(
            target_hardware_id, DiagnosticStatus.ERROR)

        # stop subscribe
        sub.unregister()

        mux_selector('input')


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestConnection)
