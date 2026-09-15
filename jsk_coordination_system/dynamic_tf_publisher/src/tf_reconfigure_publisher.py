#!/usr/bin/env python
from types import SimpleNamespace

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

from geometry_msgs.msg import TransformStamped
import tf2_ros

if ROS_VERSION == 2:
    import rclpy
    from tf_transformations import quaternion_from_euler
else:
    import rospy
    import tf
    from dynamic_reconfigure.server import Server
    from dynamic_tf_publisher.cfg import TfParameterConfig

    def quaternion_from_euler(*rpy, **kwargs):
        return tf.transformations.quaternion_from_euler(*rpy, **kwargs)

# ROS1's dynamic_reconfigure generates a self-describing Config class
# (TfParameterConfig, used below) from cfg/TfParameter.cfg at build
# time, and a Server that pushes updates to `callback` and drives the
# rqt_reconfigure GUI. ROS2 has no equivalent package/macro -- there is
# no Config class or GUI-driven callback mechanism to share here, so
# timerCallback() itself re-reads the current parameter values on every
# tick instead of relying on a push from `callback`.
_PARAM_DEFAULTS = {
    'frame_id': '', 'parent_frame_id': '',
    'x': 0.0, 'y': 0.0, 'z': 0.0,
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
}

g_node = None
g_broadcaster = None
g_frame_id = None
g_parent_frame_id = None
g_parmas = None


def callback(config, level=None):
    global g_frame_id, g_parent_frame_id, g_params
    g_frame_id = config.frame_id
    g_parent_frame_id = config.parent_frame_id
    g_params = (config.x, config.y, config.z, config.roll, config.pitch, config.yaw)
    return config


def _ros2_config():
    # Wrap the current parameter values in an object with the same
    # attribute names as ROS1's dynamic_reconfigure Config, so
    # callback() above can be reused unchanged.
    return SimpleNamespace(**ros_compat.get_params(g_node, _PARAM_DEFAULTS.keys()))


def timerCallback(event):
    if ROS_VERSION == 2:
        callback(_ros2_config())
    if g_frame_id and g_parent_frame_id and g_params:
        t = TransformStamped()
        t.header.stamp = ros_compat.stamp_now(g_node)
        t.header.frame_id = g_parent_frame_id
        t.child_frame_id = g_frame_id
        (t.transform.translation.x,
         t.transform.translation.y,
         t.transform.translation.z) = g_params[0:3]
        q = quaternion_from_euler(*g_params[3:6])
        (t.transform.rotation.x,
         t.transform.rotation.y,
         t.transform.rotation.z,
         t.transform.rotation.w) = q
        g_broadcaster.sendTransform(t)


def main():
    global g_node, g_broadcaster
    if ROS_VERSION == 2:
        rclpy.init()
        g_node = rclpy.create_node("tf_reconfigure_publisher")
        g_broadcaster = tf2_ros.TransformBroadcaster(g_node)
        ros_compat.declare_params(g_node, _PARAM_DEFAULTS)
        g_node.create_timer(0.1, lambda: timerCallback(None))
        ros_compat.spin_and_shutdown(g_node)
    else:
        rospy.init_node("tf_reconfigure_publisher")
        g_broadcaster = tf2_ros.TransformBroadcaster()
        Server(TfParameterConfig, callback)
        rospy.Timer(rospy.Duration(0.1), timerCallback)
        rospy.spin()


if __name__ == "__main__":
    main()
