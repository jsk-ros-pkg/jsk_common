#!/usr/bin/env python

import sys

import rospy
import rosbag
import tf2_ros


def main():

    rospy.init_node('static_tf_republisher')

    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) > 1:
        bagfilename = myargv[1]
    else:
        bagfilename = rospy.get_param("~file")
    mode_static = rospy.get_param('~mode_static', True)

    transforms = []
    with rosbag.Bag(bagfilename, 'r') as inputbag:
        for topic, msg, t in inputbag.read_messages('/tf_static'):
            transforms.extend(msg.transforms)

    if mode_static:
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.loginfo(
            'republish /tf_static with {} TransformStamped messages'.format(len(transforms)))
        broadcaster.sendTransform(transforms)
        rospy.spin()
    else:
        broadcaster = tf2_ros.TransformBroadcaster()
        hz = rospy.get_param('~publish_rate', 10)
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            rate.sleep()
            current_time = rospy.Time.now()
            transforms_publish = []
            for transform in transforms:
                transform.header.stamp = current_time
                transforms_publish.append(transform)
            broadcaster.sendTransform(transforms)


if __name__ == '__main__':
    main()
