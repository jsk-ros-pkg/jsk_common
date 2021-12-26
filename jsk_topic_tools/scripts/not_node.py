#!/usr/bin/env python

import rospy
import std_msgs.msg


class NOTNode(object):

    def __init__(self):
        self.pub_speech_flag = rospy.Publisher(
            '~output',
            std_msgs.msg.Bool, queue_size=1)

        self.data = None
        self.sub = rospy.Subscriber(
            '~input',
            std_msgs.msg.Bool,
            callback=self.callback,
            queue_size=1)

        rate = rospy.get_param('~rate', 100)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 100.')
            rate = 100
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def callback(self, msg):
        self.data = msg.data

    def timer_cb(self, timer):
        if self.data is None:
            return
        flag = not self.data
        self.pub_speech_flag.publish(
            std_msgs.msg.Bool(flag))


if __name__ == '__main__':
    rospy.init_node('not_node')
    node = NOTNode()
    rospy.spin()
