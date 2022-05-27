#!/usr/bin/env python

import unittest

import rospy
import tf2_ros


PKG = 'jsk_topic_tools'
NAME = 'test_static_tf_republisher'


class TestStaticTFRepublisher(unittest.TestCase):

    def __init__(self, *args):
        super(TestStaticTFRepublisher, self).__init__(*args)
        rospy.init_node(NAME)

    def test_static_tf_republisher(self):

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        frame_id_source = rospy.get_param('~frame_id_source')
        frame_id_target = rospy.get_param('~frame_id_target')

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                tf_buffer.lookup_transform(
                    frame_id_target,
                    frame_id_source,
                    rospy.Time()
                )
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(
                    'Exception while looking up tranform {}'.format(e))
                continue
        return


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestStaticTFRepublisher)
