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

from dynamic_tf_publisher.srv import * # SetDynamicTF
from geometry_msgs.msg import TransformStamped
import tf
import thread

class dynamic_tf_publisher:
    def __init__(self):
        self.cur_tf = dict()
        self.tf_sleep_time = 1.0
        self.lockobj = thread.allocate_lock()
        rospy.Service('/set_dynamic_tf', SetDynamicTF, self.set_tf)

    def publish_tf(self,frame_id):
        pose = self.cur_tf[frame_id]
        br = tf.TransformBroadcaster()
        pos = pose.transform.translation
        rot = pose.transform.rotation
        br.sendTransform((pos.x, pos.y, pos.z),
                         (rot.x, rot.y, rot.z, rot.w),
                         rospy.Time.now(),
                         pose.child_frame_id,
                         pose.header.frame_id)

    def set_tf(self,req):
        print "Latch [%s]/[%shz]"%(req.cur_tf.child_frame_id,req.freq)
        self.tf_sleep_time = 1.0/req.freq
        self.lockobj.acquire()
        self.cur_tf[req.cur_tf.child_frame_id] = req.cur_tf
        self.lockobj.release()
        self.publish_tf(req.cur_tf.child_frame_id)
        return SetDynamicTFResponse()

    def publish_and_sleep(self):
        self.lockobj.acquire()
        map(self.publish_tf, self.cur_tf)
        self.lockobj.release()
        rospy.sleep(self.tf_sleep_time)

if __name__ == "__main__":
    rospy.init_node('tf_latch_server')
    pub = dynamic_tf_publisher()
    while not rospy.is_shutdown():
        pub.publish_and_sleep()
    print "exit"

