#!/usr/bin/env python
import roslib
roslib.load_manifest('jsk_network_tools')
import rospy
from heartbeat.msg import Heartbeat, HeartbeatResponse
from std_msgs.msg import Float32

class Master():
    def __init__(self):
        rospy.init_node('heartbeat_master')
        self.pub_request = rospy.Publisher("heartbeat/request", Heartbeat)
        self.pub_round_time = rospy.Publisher("heartbeat/round", Float32)
        rospy.Subscriber("heartbeat/response", HeartbeatResponse, self.callback)
        self.rate = 1.0

        while not rospy.is_shutdown():
            msg = Heartbeat()
            msg.rate = self.rate
            msg.header.stamp = rospy.Time.now()
            self.pub_request.publish(msg)
            rospy.sleep(1.0/self.rate)

    def callback(self, data):
        now = rospy.Time.now()
        upload_time = data.header.stamp - data.heartbeat.header.stamp
        download_time = now - data.header.stamp
        round_time = now - data.heartbeat.header.stamp

        #rospy.loginfo("upload_time: %d.%09d" % (upload_time.to_sec(), upload_time.to_nsec()))
        rospy.loginfo("upload_time: %f" % (upload_time.to_sec()))
        rospy.loginfo("download_time: %f" % (download_time.to_sec()))
        rospy.loginfo("round_time: %f" % (round_time.to_sec()))
        msg = Float32(round_time.to_sec())
        self.pub_round_time.publish(msg)

        #rospy.loginfo("download_time: %d.%09d" % (download_time.secs, download_time.nsecs))
        #rospy.loginfo("header, %s" % data.heartbeat.header.stamp)
        #rospy.loginfo("%09d" % upload_time.to_nsec())

if __name__ == '__main__':
    Master()
