#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_topic_tools import jsk_logwarn
from roslib.message import get_message_class
import rosgraph
import rospy
from rospy.msg import AnyMsg


class DelayTimestamp(object):

    def __init__(self):
        rospy.init_node('delay_timestamp')
        jsk_logwarn('This node is mainly designed for TEST. '
                    'Take care if you use this in your actual system.')
        self._master = rosgraph.Master(rospy.get_name())
        self._delay = rospy.get_param('~delay')
        self._sub = rospy.Subscriber('~input', AnyMsg, self._cb)
        self._unknown_topic_type = True

    def _cb(self, msg):
        if self._unknown_topic_type:
            anymsg = msg
            # get message class from input topic
            topic_types = self._master.getTopicTypes()
            msg_name = [ty for tp, ty in topic_types
                        if tp == self._sub.name][0]
            msg_class = get_message_class(msg_name)
            self._unknown_topic_type = False
            # construct publisher
            self._pub = rospy.Publisher('~output', msg_class, queue_size=10)
            # reconstruct subscriber
            self._sub.unregister()
            self._sub = rospy.Subscriber('~input', msg_class, self._cb)
            # AnyMsg -> input message
            msg = msg_class().deserialize(anymsg._buff)
        # created delayed msg
        msg.header.stamp += rospy.Duration(self._delay)
        while msg.header.stamp > rospy.Time.now():
            rospy.sleep(0.001)
        self._pub.publish(msg)


if __name__ == '__main__':
    dl = DelayTimestamp()
    rospy.spin()
