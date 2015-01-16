#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import Heartbeat, HeartbeatResponse
from std_msgs.msg import Float32

class Master():
    def __init__(self):
        rospy.init_node('heartbeat_master')
        self.rate = rospy.get_param("~rate", 1.0)
        self.pub_request = rospy.Publisher("heartbeat/request", Heartbeat)
        self.pub_upload_time = rospy.Publisher("heartbeat/upload_time", Float32)
        self.pub_download_time = rospy.Publisher("heartbeat/download_time", Float32)
        self.pub_round_time = rospy.Publisher("heartbeat/round_time", Float32)
        rospy.Subscriber("heartbeat/response", HeartbeatResponse, self.callback)

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

        rospy.loginfo("upload_time: %f" % (upload_time.to_sec()))
        rospy.loginfo("download_time: %f" % (download_time.to_sec()))
        rospy.loginfo("round_time: %f" % (round_time.to_sec()))

        #publish time
        upload_time_msg = Float32(upload_time.to_sec())
        download_time_msg = Float32(download_time.to_sec())
        round_time_msg = Float32(round_time.to_sec())

        self.pub_upload_time.publish(upload_time_msg)
        self.pub_download_time.publish(download_time_msg)
        self.pub_round_time.publish(round_time_msg)

if __name__ == '__main__':
    Master()
