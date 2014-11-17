#!/usr/bin/env python

import rospy
import tf
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
        g_broadcaster.sendTransform(
            g_params[0:3],
            tf.transformations.quaternion_from_euler(*g_params[3:6]),
            rospy.Time.now(),
            g_frame_id,
            g_parent_frame_id)

def main():
    global g_broadcaster
    g_broadcaster = tf.TransformBroadcaster()
    srv = Server(TfParameterConfig, callback)
    rospy.Timer(rospy.Duration(0.1), timerCallback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("tf_reconfigure_publisher")
    main()
