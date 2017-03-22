#!/usr/bin/env python

import argparse
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import rospy
from std_msgs.msg import Header
import tf


class TFToTransform(object):
    def __init__(self, parent_frame_id, child_frame_id, duration, rate):
        self.duration = rospy.get_param('~duration', duration)
        self.parent_frame_id = rospy.get_param('~parent_frame_id',
                                               parent_frame_id)
        self.child_frame_id = rospy.get_param('~child_frame_id',
                                              child_frame_id)
        rate = rospy.get_param('~rate', rate)
        self.pub = rospy.Publisher('~output', TransformStamped, queue_size=1)
        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(1. / rate), self._publish_transform)

    def _publish_transform(self, event):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(
                self.parent_frame_id,
                self.child_frame_id,
                now,
                timeout=rospy.Duration(self.duration)
                )
        except Exception:
            rospy.logerr(
                "cannot get transform from '%s' to '%s' in '%.2f' [s]" %
                (self.parent_frame_id, self.child_frame_id, self.duration))
            return

        trans, rot = self.listener.lookupTransform(
            self.parent_frame_id,
            self.child_frame_id,
            now
            )
        tf_msg = Transform(
            translation=Vector3(
                x=trans[0],
                y=trans[1],
                z=trans[2]
                ),
            rotation=Quaternion(
                x=rot[0],
                y=rot[1],
                z=rot[2],
                w=rot[3]
                )
            )
        tf_stamped_msg = TransformStamped(
            header=Header(
                stamp=now,
                frame_id=self.parent_frame_id
                ),
            child_frame_id=self.child_frame_id,
            transform=tf_msg
            )
        self.pub.publish(tf_stamped_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('parent_frame_id', nargs='?',
                        help='parent frame id', default=None)
    parser.add_argument('child_frame_id', nargs='?',
                        help='child frame id', default=None)
    parser.add_argument('--duration', '-d', type=float, default=1,
                        help='Duration to resolve tf. default: 1 [s]')
    parser.add_argument('--rate', '-r', type=float, default=1,
                        help='Rate of publication. default: 1 [hz]')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('tf_to_transform')
    tf_pub = TFToTransform(
        args.parent_frame_id,
        args.child_frame_id,
        args.duration,
        args.rate,
    )
    rospy.spin()
