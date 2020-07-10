#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def cb(event):
    stamp = rospy.Time.now()
    try:
        listener.waitForTransform(src_frame, dst_frame, stamp,
                                timeout=rospy.Duration(1))
    except Exception as e:
        rospy.logerr(e)
        return

    dst_pose = listener.lookupTransform(src_frame, dst_frame, stamp)

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = src_frame
    pose_msg.header.stamp = stamp
    pose_msg.pose.position.x = dst_pose[0][0]
    pose_msg.pose.position.y = dst_pose[0][1]
    pose_msg.pose.position.z = dst_pose[0][2]
    pose_msg.pose.orientation.x = dst_pose[1][0]
    pose_msg.pose.orientation.y = dst_pose[1][1]
    pose_msg.pose.orientation.z = dst_pose[1][2]
    pose_msg.pose.orientation.w = dst_pose[1][3]

    pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('tf_to_pose')
    pub = rospy.Publisher('~output', PoseStamped, queue_size=1)
    src_frame = rospy.get_param('~src_frame')
    dst_frame = rospy.get_param('~dst_frame')
    rate = rospy.get_param('~rate', 1.)
    listener = tf.TransformListener()
    timer = rospy.Timer(rospy.Duration(1.0 / rate), cb)
    rospy.spin()
