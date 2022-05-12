#!/usr/bin/env python

# https://github.com/start-jsk/jsk_apc/blob/master/jsk_arc2017_baxter/node_scripts/sanity_check_for_pick.py # NOQA

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from jsk_tools.sanity_lib import checkNodeState, checkTopicIsPublished
import rospy


class SanityDiagnostics(object):
    """
    This node publishes essential robot topic and node status to /diagnostics.

    The yaml file is like the following:
    topics:
      - /kinect_head/rgb/image_raw
      - /tf
    nodes:
      - /kinect_head/kinect_head_nodelet_manager
      - /respeaker_node
    """
    def __init__(self):
        duration = rospy.get_param('~duration', 60)
        # Get sanity target topic and node names
        topics = rospy.get_param('~topics', [])
        nodes = rospy.get_param('~nodes', [])
        # Set diagnostic updater for each topic and node
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")
        for topic_name in topics:
            def check_topic(stat):
                self.check_topic(stat, topic_name)
            self.updater.add(topic_name, check_topic)
        for node_name in nodes:
            def check_node(stat):
                self.check_node(stat, node_name)
            self.updater.add(node_name, check_node)
        # Timer to call updater
        self.timer = rospy.Timer(
            rospy.Duration(duration), self.check_sanity)

    def check_sanity(self, event):
        self.updater.update()

    def check_topic(self, stat, topic_name):
        topic_state = checkTopicIsPublished(topic_name)
        if topic_state:
            stat.summary(DiagnosticStatus.OK, 'Topic is published')
        else:
            stat.summary(DiagnosticStatus.OK, 'Topic is not published')
        return stat

    def check_node(self, stat, node_name):
        node_state = checkNodeState(node_name, needed=True)
        if node_state:
            stat.summary(DiagnosticStatus.OK, 'Node is alive')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Node is dead')
        return stat


if __name__ == '__main__':
    rospy.init_node('sanity_diagnostics')
    SanityDiagnostics()
    rospy.spin()
