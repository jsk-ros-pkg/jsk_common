#!/usr/bin/env python

PKG = 'tf_relay'
NAME = 'tf_relay_test'

import rostest
import unittest
import sys

import rospy
import tf2_ros

class TestTFRelay(unittest.TestCase):

    def test_tf_relay(self):

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        frame_id_a = 'frame_a'
        frame_id_c = 'frame_c'
        frame_id_relayed = 'frame_relayed'

        transform_a_to_c = tf_buffer.lookup_transform(
                            frame_id_a,
                            frame_id_c,
                            rospy.Time(),
                            rospy.Duration(10)
                            )
        transform_a_to_relayed = tf_buffer.lookup_transform(
                            frame_id_a,
                            frame_id_relayed,
                            rospy.Time(),
                            rospy.Duration(10)
                            )

        self.assertEqual(transform_a_to_c.transform.translation.x, 
                         transform_a_to_relayed.transform.translation.x)
        self.assertEqual(transform_a_to_c.transform.translation.y, 
                         transform_a_to_relayed.transform.translation.y)
        self.assertEqual(transform_a_to_c.transform.translation.z, 
                         transform_a_to_relayed.transform.translation.z)
        self.assertEqual(transform_a_to_c.transform.rotation.x, 
                         transform_a_to_relayed.transform.rotation.x)
        self.assertEqual(transform_a_to_c.transform.rotation.y, 
                         transform_a_to_relayed.transform.rotation.y)
        self.assertEqual(transform_a_to_c.transform.rotation.z, 
                         transform_a_to_relayed.transform.rotation.z)
        self.assertEqual(transform_a_to_c.transform.rotation.w, 
                         transform_a_to_relayed.transform.rotation.w)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestTFRelay, sys.argv)
