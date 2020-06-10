#!/usr/bin/env python
# -*-coding: utf-8 -*-

import unittest
import rospy
from numpy import pi, sqrt, cos, sin
from geometry_msgs.msg import PoseStamped

PKG = 'jsk_topic_tools'
NAME = 'test_pose_stamped_publisher'

class TestPoseStampedPublisher(unittest.TestCase):
    def __init__(self, *args):
        super(TestPoseStampedPublisher, self).__init__(*args)
        rospy.init_node(NAME)
        self.poses = {}

    def test_pose(self):
        '''
        Rotation represented by (roll, pitch, yaw) = (-pi/2, pi/2, pi/2) [rad] in static xyz
        is equivalent to the pi[rad] rotation around the axis (-1/sqrt(2), 0, 1/sqrt(2)).
        Rotation represented by the angle theta[rad] around the axis r=(rx, ry, rz) is represented by
        Quaternion q = (rx*sin(thta/2), ry*sin(theta/2), rz*sin(theta/2), cos(theta/2)).
        see http://docs.ros.org/jade/api/tf/html/python/transformations.html
        '''
        topic_name = 'pose'
        rospy.Subscriber(topic_name, PoseStamped, lambda msg: self.callback(topic_name, msg))
        if not self.wait_until_topic_ready(topic_name):
            raise ValueError('Not Found Topic: {}'.format(topic_name))
        self.assert_topic(topic_name, [1,2,3], [-1./sqrt(2)*sin(pi/2), 0*sin(pi/2), 1./sqrt(2)*sin(pi/2), cos(pi/2)], 'world')

    def callback(self, topic_name, msg):
        self.poses[topic_name] = msg

    def wait_until_topic_ready(self, topic_name, timeout=3):
        """Wait until the topic is subscribed
        Args:
            topic_name (str): Name of topic to subscribe.
            timeout (float): Duration to timeout when the topic won't be subscribed.
        Returns:
            bool: if the topic has been subscribed.
        """
        start = rospy.Time.now()
        while (self.poses.get(topic_name) is None) and (rospy.Time.now() - start < rospy.Duration(timeout)):
            rospy.sleep(1)
        return self.poses.get(topic_name) is not None

    def assert_topic(self, topic_name, p, q, frame_id):
        """Assert the data in topic is equal to the given data.
        Args:
            topic_name (str): Name of the topic to check.
            p ([float, float, float]): Expected Position
            q ([float, float, float, float]): Expected Quaternion
            frame_id (str): Expected frame_id
        """
        self.assertEqual(self.poses[topic_name].header.frame_id, frame_id)
        self.assertAlmostEqual(self.poses[topic_name].pose.position.x, p[0])
        self.assertAlmostEqual(self.poses[topic_name].pose.position.y, p[1])
        self.assertAlmostEqual(self.poses[topic_name].pose.position.z, p[2])
        self.assertAlmostEqual(self.poses[topic_name].pose.orientation.x, q[0])
        self.assertAlmostEqual(self.poses[topic_name].pose.orientation.y, q[1])
        self.assertAlmostEqual(self.poses[topic_name].pose.orientation.z, q[2])
        self.assertAlmostEqual(self.poses[topic_name].pose.orientation.w, q[3])
        self.assertAlmostEqual(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2,  1)

if __name__=='__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestPoseStampedPublisher)
