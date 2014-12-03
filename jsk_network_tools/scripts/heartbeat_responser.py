#!/usr/bin/env python
import rospy
from jsk_network_tools.msg import Heartbeat, HeartbeatResponse

class Responser():
    def __init__(self):
        rospy.init_node('heartbeat_responser')
        self.pub_response = rospy.Publisher("heartbeat/response", HeartbeatResponse)
        rospy.Subscriber("heartbeat/request", Heartbeat, self.callback)
        rospy.spin()

    def callback(self, heartbeat):
        res = HeartbeatResponse()
        res.header.stamp = rospy.Time.now()
        res.heartbeat = heartbeat
        self.pub_response.publish(res)
        rospy.loginfo("respond to msg published %s.%s" % (heartbeat.header.stamp.secs, heartbeat.header.stamp.nsecs))


if __name__ == '__main__':
    Responser()
