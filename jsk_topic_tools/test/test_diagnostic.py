#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import roslib
import rospy
from topic_tools.srv import MuxSelect

from test_connection import TestConnection


PKG = 'jsk_topic_tools'
NAME = 'test_diagnostic_transport'


class TestDiagnosticTransport(TestConnection):

    def __init__(self, *args):
        super(TestDiagnosticTransport, self).__init__(*args)

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
    rostest.rosrun(PKG, NAME, TestDiagnosticTransport)
