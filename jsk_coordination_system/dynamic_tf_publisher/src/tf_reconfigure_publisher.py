#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from dynamic_reconfigure.server import Server
from dynamic_tf_publisher.cfg import TfParameterConfig

g_broadcaster = None
g_frame_id = None
g_parent_frame_id = None
g_parmas = None
def callback(config, level):
    global g_broadcaster, g_frame_id, g_parent_frame_id, g_params
    g_frame_id = config.frame_id
    g_parent_frame_id = config.parent_frame_id
    g_params = (config.x, config.y, config.z, config.roll, config.pitch, config.yaw)
    return config

def timerCallback(event):
    global g_broadcaster, g_frame_id, g_parent_frame_id, g_params
    if g_frame_id and g_parent_frame_id and g_params:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = g_parent_frame_id
        t.child_frame_id = g_frame_id
        (t.transform.translation.x,
         t.transform.translation.y,
         t.transform.translation.z) = g_params[0:3]
        q = tf.transformations.quaternion_from_euler(*g_params[3:6])
        (t.transform.rotation.x,
         t.transform.rotation.y,
         t.transform.rotation.z,
         t.transform.rotation.w) = q
        g_broadcaster.sendTransform(t)

def main():
    global g_broadcaster
    g_broadcaster = tf2_ros.TransformBroadcaster()
    srv = Server(TfParameterConfig, callback)
    rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("tf_reconfigure_publisher")
    main()
