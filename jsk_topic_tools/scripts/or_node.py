#!/usr/bin/env python

import rospy
import std_msgs.msg


class ORNode(object):

    def __init__(self):
        self.pub_speech_flag = rospy.Publisher(
            '~output',
            std_msgs.msg.Bool, queue_size=1)

        self.data = {}

        self.sub01 = rospy.Subscriber(
            '~input01',
            std_msgs.msg.Bool,
            callback=lambda msg: self.callback('sub01', msg),
            queue_size=1)

        self.sub02 = rospy.Subscriber(
            '~input02',
            std_msgs.msg.Bool,
            callback=lambda msg: self.callback('sub02', msg),
            queue_size=1)

        rospy.Timer(rospy.Duration(0.01), self.timer_cb)

    def callback(self, topic_name, msg):
        self.data[topic_name] = msg.data

    def timer_cb(self, timer):
        if len(self.data) == 0:
            return
        flag = any(self.data.values())
        self.pub_speech_flag.publish(
            std_msgs.msg.Bool(flag))


if __name__ == '__main__':
    rospy.init_node('or_node')
    node = ORNode()
    rospy.spin()
