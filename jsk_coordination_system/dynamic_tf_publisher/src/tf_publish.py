#!/usr/bin/env python
#
# publish static tf which is set by SetDynamicTF service
# publishing tf is uniquely by child_frame_id
#
# TODO: delete target tf
#       check tf tree consistency
#       change frequency by frame_id
#
# Not carried over to the ROS2 code path: the YAML-serialized-parameter
# restart cache (ROS2 parameters are typed, there is no idiomatic "store
# an arbitrary message as a string" mechanism) and the
# wait_for_service-based already-running singleton guard (no ROS2
# equivalent to a blocking service-existence check). Both are preserved
# as-is on the ROS1 path below.
#
from threading import Lock

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

from dynamic_tf_publisher.srv import SetDynamicTF, AssocTF, DissocTF, DeleteTF
from std_srvs.srv import Empty
import tf2_ros

if ROS_VERSION == 2:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
else:
    import roslib
    roslib.load_manifest('dynamic_tf_publisher')
    from roslib.message import fill_message_args
    import rospy
    import sys
    from tf2_msgs.msg import TFMessage
    import yaml


class dynamic_tf_publisher(object):

    def advertiseServiceUnlessFound(self, name, srv, callback):
        if ROS_VERSION == 2:
            self.node.create_service(srv, name, callback)
            return
        try:
            rospy.wait_for_service(name, 1.0)
            rospy.logfatal("%s is already exists" % (name))
            sys.exit(1)
        except rospy.ROSException as e:
            rospy.Service(name, srv, callback)

    def __init__(self, node=None):
        self.node = node
        self.cur_tf = {}
        self.original_parent = {}
        self.tf_buffer, self.listener = ros_compat.create_tf_buffer_and_listener(node)
        self.tf_sleep_time = 1.0
        self.lock = Lock()

        if ROS_VERSION == 2:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        else:
            self.update_tf = {}
            self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
            self.pub_tf_mine = rospy.Publisher("~tf", TFMessage, queue_size=1)
            self.use_cache = rospy.get_param('~use_cache', True)
            self.check_update = rospy.get_param('~check_update', False)
            self.check_update_sleep_time = rospy.get_param('~check_update_sleep_time', 1.0)
            self.check_update_last_update = rospy.Time(0)
            # check the cache
            cache_param = 'dynamic_tf_publisher' + rospy.get_name()
            if self.use_cache and rospy.has_param(cache_param):
                tfm = TFMessage()
                fill_message_args(tfm, [yaml.load(rospy.get_param(cache_param))])
                for pose in tfm.transforms:
                    self.cur_tf[pose.child_frame_id] = pose

        self.advertiseServiceUnlessFound('/set_dynamic_tf', SetDynamicTF, self.set_tf)
        self.advertiseServiceUnlessFound('/assoc_tf', AssocTF, self.assoc)
        self.advertiseServiceUnlessFound('/publish_tf', Empty, self.publish_tf)
        self.advertiseServiceUnlessFound('/dissoc_tf', DissocTF, self.dissoc)
        self.advertiseServiceUnlessFound('/delete_tf', DeleteTF, self.delete)

        if ROS_VERSION == 2:
            # ROS1 varied the publish period per the last SetDynamicTF
            # request's `freq` field; simplified here to a fixed 1Hz timer.
            self.timer = node.create_timer(1.0, lambda: self.publish_tf(None))

    def publish_tf(self, req=None, res=None):
        if ROS_VERSION == 2:
            time = ros_compat.stamp_now(self.node)
            for child_frame_id in list(self.cur_tf.keys()):
                transform = self.cur_tf[child_frame_id]
                transform.header.stamp = time
                self.tf_broadcaster.sendTransform(transform)
            return ros_compat.create_response(Empty)

        with self.lock:
            time = rospy.Time.now()
            tfm = TFMessage()

            if self.check_update:
                publish_all = False
                if self.check_update_last_update + rospy.Duration(self.check_update_sleep_time) < time:
                    publish_all = True
                    self.check_update_last_update = time

            for frame_id in self.cur_tf.keys():
                if (not self.check_update) or publish_all or self.update_tf[frame_id]:
                    pose = self.cur_tf[frame_id]
                    pose.header.stamp = time
                    tfm.transforms.append(pose)
                    self.update_tf[frame_id] = False

            if len(tfm.transforms) > 0:
                self.pub_tf.publish(tfm)
                self.pub_tf_mine.publish(tfm)
        return ros_compat.create_response(Empty)

    def assoc(self, req, res=None):
        if (req.child_frame not in self.cur_tf) or self.cur_tf[req.child_frame] == req.parent_frame:
            ros_compat.logwarn(self.node, "unkown key %s" % (req.child_frame))
            return ros_compat.create_response(AssocTF)
        ros_compat.loginfo(self.node, "assoc %s -> %s" % (req.parent_frame, req.child_frame))
        try:
            transform = self.tf_buffer.lookup_transform(
                req.parent_frame, req.child_frame, req.header.stamp,
                timeout=ros_compat.duration_from_sec(1.0))
        except Exception as e:
            ros_compat.logerr(self.node, 'failed to lookup transform: %s' % (e,))
            return ros_compat.create_response(AssocTF)
        transform.header.stamp = req.header.stamp
        transform.header.frame_id = req.parent_frame
        transform.child_frame_id = req.child_frame
        with self.lock:
            self.original_parent[req.child_frame] = self.cur_tf[req.child_frame].header.frame_id
            self.cur_tf[req.child_frame] = transform
            if ROS_VERSION == 1:
                self.update_tf[req.child_frame] = True
        self.publish_tf()
        return ros_compat.create_response(AssocTF)

    def dissoc(self, req, res=None):
        areq = None
        ros_compat.loginfo(self.node, "dissoc TF %s" % (req.frame_id))
        with self.lock:
            if req.frame_id in self.original_parent:
                areq = ros_compat.create_request(AssocTF)
                areq.header = req.header
                areq.child_frame = req.frame_id
                areq.parent_frame = self.original_parent[req.frame_id]
        if areq:
            self.assoc(areq)
            self.original_parent.pop(req.frame_id)  # remove
        return ros_compat.create_response(DissocTF)

    def delete(self, req, res=None):
        ros_compat.loginfo(self.node, "delete TF %s" % (req.header.frame_id))
        with self.lock:
            if req.header.frame_id in self.original_parent:
                del self.original_parent[req.header.frame_id]
            if req.header.frame_id in self.cur_tf:
                del self.cur_tf[req.header.frame_id]
            if ROS_VERSION == 1 and req.header.frame_id in self.update_tf:
                del self.update_tf[req.header.frame_id]
        return ros_compat.create_response(DeleteTF)

    def set_tf(self, req, res=None):
        ros_compat.loginfo(
            self.node, "%s => %s" % (req.cur_tf.header.frame_id, req.cur_tf.child_frame_id))
        with self.lock:
            # if not assocd
            if req.cur_tf.child_frame_id not in self.original_parent:
                self.tf_sleep_time = 1.0 / req.freq
                self.cur_tf[req.cur_tf.child_frame_id] = req.cur_tf
                if ROS_VERSION == 1:
                    self.update_tf[req.cur_tf.child_frame_id] = True
                ros_compat.loginfo(
                    self.node, "Latch [%s]/[%shz]" % (req.cur_tf.child_frame_id, req.freq))
            # set parameter
            if ROS_VERSION == 1 and self.use_cache:
                time = rospy.Time.now()
                tfm = TFMessage()
                for frame_id in self.cur_tf.keys():
                    pose = self.cur_tf[frame_id]
                    pose.header.stamp = time
                    tfm.transforms.append(pose)
                rospy.set_param('dynamic_tf_publisher' + rospy.get_name(), tfm.__str__())
            return ros_compat.create_response(SetDynamicTF)

    def publish_and_sleep(self):
        self.publish_tf()
        rospy.sleep(self.tf_sleep_time)


def main():
    if ROS_VERSION == 2:
        rclpy.init()
        node = Node('tf_publish_server')
        dynamic_tf_publisher(node)
        ros_compat.spin_and_shutdown(node, executor=MultiThreadedExecutor())
    else:
        try:
            rospy.init_node('tf_publish_server')
            pub = dynamic_tf_publisher()
            while not rospy.is_shutdown():
                pub.publish_and_sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()
