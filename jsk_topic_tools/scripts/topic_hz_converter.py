#!/usr/bin/env python

import rospy
import rosgraph
from rospy.msg import AnyMsg
from roslib.message import get_message_class


class TopicHzConverter:
    """
    This class converts topic hz.
    """
    def __init__(self):
        self._master = rosgraph.Master(rospy.get_name())
        self._topic_hz = rospy.get_param('~desired_topic_hz')
        self._sub = rospy.Subscriber("~input", AnyMsg, self._callback)
        self._timer_event = rospy.Timer(
            rospy.Duration(1.0/float(self._topic_hz)), self._hz_converter,
            oneshot=False)
        self._unknown_topic_type = True
        rospy.loginfo('topic hz convert starting')

    def _callback(self, msg):
        if self._unknown_topic_type:
            anymsg = msg
            topic_types = self._master.getTopicTypes()
            msg_name = [ty for tp, ty in topic_types
                        if tp == self._sub.name][0]
            self._msg_class = get_message_class(msg_name)
            # construct publisher
            self._pub = rospy.Publisher(
                "~output", self._msg_class, queue_size=1)
            # reconstruct subscriber for specific msg class
            self._sub.unregister()
            self._sub = rospy.Subscriber(
                '~input', self._msg_class, self._callback)
            msg = self._msg_class().deserialize(anymsg._buff)
            self._unknown_topic_type = False
        # update topic
        self._output = msg

    def _hz_converter(self, event):
        if self._unknown_topic_type:
            pass
        else:
            # if msg has header, update timestamp
            if self._msg_class._has_header:
                self._output.header.stamp = rospy.Time.now()
            self._pub.publish(self._output)


if __name__ == '__main__':
    rospy.init_node('topic_hz_converter', anonymous=True)
    topic_hz_converter = TopicHzConverter()
    rospy.spin()
