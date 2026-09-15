#!/usr/bin/env python
import sys

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

from dynamic_tf_publisher.srv import SetDynamicTF
from geometry_msgs.msg import Vector3, Quaternion

if ROS_VERSION == 2:
    import rclpy
    from tf_transformations import quaternion_from_euler
else:
    import roslib
    roslib.load_manifest('dynamic_tf_publisher')
    import tf

    def quaternion_from_euler(*rpy, **kwargs):
        return tf.transformations.quaternion_from_euler(*rpy, **kwargs)


def set_tf(node, pos, q, pa, ch, freq):
    try:
        request = ros_compat.create_request(SetDynamicTF)
        request.freq = freq
        request.cur_tf.header.frame_id = pa
        request.cur_tf.child_frame_id = ch
        request.cur_tf.transform.translation = Vector3(x=pos[0], y=pos[1], z=pos[2])
        request.cur_tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        print(request.cur_tf)
        ros_compat.call_service(node, '/set_dynamic_tf', SetDynamicTF, request)
        return
    except Exception as e:
        print("Service call failed: %s" % e)


def isfloat(s):
    try:
        float(s)
        return True
    except Exception:
        return False


if __name__ == "__main__":
    node = None
    if ROS_VERSION == 2:
        rclpy.init()
        node = rclpy.create_node('tf_set')
    try:
        pos = list(map(float, sys.argv[1:4]))
        if isfloat(sys.argv[7]):  # quaternion
            q = list(map(float, sys.argv[4:8]))
            rest = sys.argv[8:]
        else:  # rpy
            rpy = list(map(float, sys.argv[4:7]))
            # rpy angle is in "zyx" axis
            q = quaternion_from_euler(*rpy, axes="rzyx")
            print(q)
            rest = sys.argv[7:]
        pa = rest[0]
        ch = rest[1]
        hz = 1000.0 / float(rest[2])
        set_tf(node, pos, q, pa, ch, hz)
    except Exception as e:
        print(e)
        print(sys.argv)
        print("args: x y z (r p y)|(x y z w) parent child msec")
    finally:
        if ROS_VERSION == 2:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    sys.exit(0)
