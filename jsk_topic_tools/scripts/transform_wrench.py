#!/usr/bin/env python

from geometry_msgs.msg import WrenchStamped
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix as quaternion2matrix

from jsk_topic_tools import ConnectionBasedTransport


class WrenchTransform(ConnectionBasedTransport):

    def __init__(self):
        super(WrenchTransform, self).__init__()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._duration_timeout = rospy.get_param('~duration_timeout', 0.05)
        self.target_frame_id = rospy.get_param('~target_frame_id')
        self.pub = self.advertise('~output', WrenchStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input',
            WrenchStamped, queue_size=1,
            callback=self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame_id,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rospy.Duration(self._duration_timeout)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return
        rot = transform.transform.rotation
        trans = transform.transform.translation
        matrix = quaternion2matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        force = np.dot(matrix,
                       [msg.wrench.force.x,
                        msg.wrench.force.y,
                        msg.wrench.force.z])
        torque = np.dot(matrix,
                        [msg.wrench.torque.x,
                         msg.wrench.torque.y,
                         msg.wrench.torque.z]) + np.cross(
                             [trans.x, trans.y, trans.z], force)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
        msg.header.frame_id = self.target_frame_id
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('transform_wrench')
    wrench_transform = WrenchTransform()  # NOQA
    rospy.spin()
