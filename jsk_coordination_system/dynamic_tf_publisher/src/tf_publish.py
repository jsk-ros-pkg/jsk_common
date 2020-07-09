#!/usr/bin/env python
#
# publish static tf which is set by SetDynamicTF service
# publishing tf is uniquely by child_frame_id
#
# TODO: delete target tf
#       check tf tree consistency
#       change frequency by frame_id
#
import roslib; roslib.load_manifest('dynamic_tf_publisher')
import rospy
import sys
from dynamic_tf_publisher.srv import * # SetDynamicTF
from geometry_msgs.msg import TransformStamped,Quaternion,Vector3
from std_srvs.srv import Empty, EmptyResponse
import tf
import tf.msg
import thread
from threading import Lock
import yaml

class dynamic_tf_publisher:
    def advertiseServiceUnlessFound(self, name, srv, callback):
        try:
            rospy.wait_for_service(name, 1.0)
            rospy.logfatal("%s is already exists" % (name))
            sys.exit(1)
        except rospy.ROSException as e:
            rospy.Service(name, srv, callback)
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)
        self.pub_tf_mine = rospy.Publisher("~tf", tf.msg.tfMessage, queue_size=1)
        self.cur_tf = dict()
        self.original_parent = dict()
        self.update_tf = dict()
        self.listener = tf.TransformListener()
        self.tf_sleep_time = 1.0
        self.lock = Lock()

        self.use_cache = rospy.get_param('~use_cache', True)
        self.check_update = rospy.get_param('~check_update', False)
        self.check_update_sleep_time = rospy.get_param('~check_update_sleep_time', 1.0)
        self.check_update_last_update = rospy.Time(0)
        # check the cache
        if self.use_cache and rospy.has_param('dynamic_tf_publisher'+rospy.get_name()) :
            tfm = tf.msg.tfMessage()
            roslib.message.fill_message_args(tfm,[yaml.load(rospy.get_param('dynamic_tf_publisher'+rospy.get_name()))])
            for pose in tfm.transforms :
                self.cur_tf[pose.child_frame_id] = pose
        self.advertiseServiceUnlessFound('/set_dynamic_tf', SetDynamicTF, self.set_tf)
        self.advertiseServiceUnlessFound('/assoc_tf', AssocTF, self.assoc)
        self.advertiseServiceUnlessFound('/publish_tf', Empty, self.publish_tf)
        self.advertiseServiceUnlessFound('/dissoc_tf', DissocTF, self.dissoc)
        self.advertiseServiceUnlessFound('/delete_tf', DeleteTF, self.delete)

    def publish_tf(self, req=None):
        with self.lock:
            time = rospy.Time.now()
            tfm = tf.msg.tfMessage()

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
        return EmptyResponse()

    def assoc(self,req):
        if (not self.cur_tf.has_key(req.child_frame)) or self.cur_tf[req.child_frame] == req.parent_frame:
            rospy.logwarn("unkown key %s" % (req.child_frame))
            return AssocTFResponse()
        rospy.loginfo("assoc %s -> %s"%(req.parent_frame, req.child_frame))
        self.listener.waitForTransform(req.parent_frame,
                                       req.child_frame,
                                       req.header.stamp, rospy.Duration(1.0))
        ts = TransformStamped()
        (trans,rot) = self.listener.lookupTransform(req.parent_frame, req.child_frame, req.header.stamp)
        ts.transform.translation = Vector3(*trans)
        ts.transform.rotation = Quaternion(*rot)
        ts.header.stamp = req.header.stamp
        ts.header.frame_id = req.parent_frame
        ts.child_frame_id = req.child_frame
        with self.lock:
            self.original_parent[req.child_frame] = self.cur_tf[req.child_frame].header.frame_id
            self.cur_tf[req.child_frame] = ts
            self.update_tf[req.child_frame] = True
        self.publish_tf()
        return AssocTFResponse()

    def dissoc(self,req):
        areq = None
        rospy.loginfo("dissoc TF %s" % (req.frame_id))
        with self.lock:
            if self.original_parent.has_key(req.frame_id):
                areq = AssocTFRequest()
                areq.header = req.header
                areq.child_frame = req.frame_id
                areq.parent_frame = self.original_parent[req.frame_id]
        if areq:
            self.assoc(areq)
            self.original_parent.pop(req.frame_id) # remove 
        return DissocTFResponse()

    def delete(self,req):
        rospy.loginfo("delete TF %s"%(req.header.frame_id))
        with self.lock:
            if self.original_parent.has_key(req.header.frame_id):
                del self.original_parent[req.header.frame_id]
            if self.cur_tf.has_key(req.header.frame_id):
                del self.cur_tf[req.header.frame_id]
            if self.update_tf.has_key(req.header.frame_id):
                del self.update_tf[req.header.frame_id]
        return DeleteTFResponse()

    def set_tf(self,req):
        rospy.loginfo("%s => %s" % (req.cur_tf.header.frame_id, req.cur_tf.child_frame_id))
        with self.lock:
            # if not assocd
            if not self.original_parent.has_key(req.cur_tf.child_frame_id):
                self.tf_sleep_time = 1.0/req.freq
                self.cur_tf[req.cur_tf.child_frame_id] = req.cur_tf
                self.update_tf[req.cur_tf.child_frame_id] = True
                rospy.loginfo("Latch [%s]/[%shz]"%(req.cur_tf.child_frame_id,req.freq))
            # set parameter
            if self.use_cache:
                time = rospy.Time.now()
                tfm = tf.msg.tfMessage()
                for frame_id in self.cur_tf.keys():
                    pose = self.cur_tf[frame_id]
                    pose.header.stamp = time
                    tfm.transforms.append(pose)
                rospy.set_param('dynamic_tf_publisher'+rospy.get_name(),tfm.__str__())
            return SetDynamicTFResponse()

    def publish_and_sleep(self):
        self.publish_tf()
        rospy.sleep(self.tf_sleep_time)


if __name__ == "__main__":
    try:
        rospy.init_node('tf_publish_server')
        pub = dynamic_tf_publisher()
        while not rospy.is_shutdown():
            pub.publish_and_sleep()
    except rospy.ROSInterruptException:
        pass

