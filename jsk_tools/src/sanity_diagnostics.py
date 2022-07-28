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
        self.check_duration = rospy.get_param('~duration', 60)
        # To always display the sanity diagnostics status in rqt_robot_monitor,
        # /diagnostics should be published at more than
        # diagnostics aggregator's update frequency (Default: 1 Hz)
        # To do so, both frequency at which self.updater.update() is
        # called and diagnostic updater's ~diagnostic_period must be
        # greater than the diagnostics aggregator's update frequency.
        pub_duration = rospy.get_param('~pub_duration', 0.3)
        rospy.set_param('~diagnostic_period', pub_duration)
        # Get sanity target topic and node names
        topics = rospy.get_param('~topics', [])
        nodes = rospy.get_param('~nodes', [])
        # Set diagnostic updater for each topic and node
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")
        self.last_time_check_topic = {}
        self.last_time_check_node = {}
        # Topic and node status
        self.topic_state = {}
        self.node_state = {}
        for topic_name in topics:
            self.updater.add(
                topic_name,
                lambda stat, tn=topic_name: self.check_topic(stat, tn))
            self.last_time_check_topic[topic_name] = rospy.Time.now()
            self.topic_state[topic_name] = True
        for node_name in nodes:
            self.updater.add(
                node_name,
                lambda stat, nn=node_name: self.check_node(stat, nn))
            self.last_time_check_node[node_name] = rospy.Time.now()
            self.node_state[node_name] = True
        # Timer to call updater
        self.timer = rospy.Timer(
            rospy.Duration(pub_duration), self.pub_diagnostics)

    def pub_diagnostics(self, event):
        self.updater.update()

    def check_topic(self, stat, topic_name):
        elapsed_time = rospy.Time.now() - self.last_time_check_topic[topic_name]
        if elapsed_time.secs > self.check_duration:
            # Assume that topic is published at more than (1.0 / timeout) Hz
            self.topic_state[topic_name] = checkTopicIsPublished(
                topic_name, timeout=10)
            self.last_time_check_topic[topic_name] = rospy.Time.now()
        if self.topic_state[topic_name]:
            stat.summary(DiagnosticStatus.OK, 'Topic is published')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Topic is not published')
        return stat

    def check_node(self, stat, node_name):
        elapsed_time = rospy.Time.now() - self.last_time_check_node[node_name]
        if elapsed_time.secs > self.check_duration:
            # Assume that topic is published at more than (1.0 / timeout) Hz
            self.node_state[node_name] = checkNodeState(node_name, needed=True)
            self.last_time_check_node[node_name] = rospy.Time.now()
        if self.node_state[node_name]:
            stat.summary(DiagnosticStatus.OK, 'Node is alive')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Node is dead')
        return stat


if __name__ == '__main__':
    rospy.init_node('sanity_diagnostics')
    SanityDiagnostics()
    rospy.spin()
