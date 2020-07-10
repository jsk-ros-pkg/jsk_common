#!/usr/bin/env python
import roslib; roslib.load_manifest('dynamic_tf_publisher')

import sys
import rospy
import tf
from dynamic_tf_publisher.srv import SetDynamicTF
from geometry_msgs.msg import *

def set_tf(pos, q, parent, child, freq):
    rospy.wait_for_service('/set_dynamic_tf')
    try:
        client = rospy.ServiceProxy('/set_dynamic_tf', SetDynamicTF)
        pose = TransformStamped()
        pose.header.frame_id = pa
        pose.child_frame_id = ch
        pose.transform.translation = Vector3(*pos)
        pose.transform.rotation = Quaternion(*q)
        print(pose)
        res = client(freq, pose)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def isfloat(str):
    try:
        float(str)
        return True
    except Exception as e:
        return False

if __name__ == "__main__":
    try:
        pos = map(float,sys.argv[1:4])
        if isfloat(sys.argv[7]): # quaternion
            q = map(float,sys.argv[4:8])
            rest = sys.argv[8:]
        else: # rpy
            rpy = map(float,sys.argv[4:7])
            # rpy angle is in "zyx" axis
            q = tf.transformations.quaternion_from_euler(*rpy, axes="rzyx")
            print(q)
            rest = sys.argv[7:]
        pa = rest[0]
        ch = rest[1]
        hz = 1000.0 / float(rest[2])
        set_tf(pos, q, pa, ch, hz)
    except Exception as e:
        print(sys.argv)
        print("args: x y z (r p y)|(x y z w) parent child msec")
    sys.exit(0)
