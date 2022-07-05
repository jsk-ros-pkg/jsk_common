#!/usr/bin/env python

import rospy
import rostopic
import sys


class ExtractSubtopic(object):

    def __init__(self):

        topic = rospy.resolve_name('~input')
        real_msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic)
        rospy.logdebug('topic: {}, real_topic: {}'.format(topic, real_topic))
        rospy.logdebug('Wating for a message from real_topic for publisher message type.')
        msg = rospy.wait_for_message(real_topic, real_msg_class)
        msg_subtopic = msg_eval(msg)
        sub_msg_class = type(msg_subtopic)
        rospy.logdebug('publisher message type: {}'.format(sub_msg_class))
        try:
            self.msg_eval = msg_eval
            self.pub = rospy.Publisher('~output', sub_msg_class, queue_size=1)
            self.sub = rospy.Subscriber(real_topic, real_msg_class, self.callback)
        except ValueError:
            rospy.logerr('Invalid subtopic message type: {}'.format(sub_msg_class))
            sys.exit(1)
        rospy.logdebug('initialized')

    def callback(self, msg):

        msg_subtopic = self.msg_eval(msg)
        self.pub.publish(msg_subtopic)


if __name__ == '__main__':

    rospy.init_node('extract_subtopic')
    node = ExtractSubtopic()
    rospy.spin()
