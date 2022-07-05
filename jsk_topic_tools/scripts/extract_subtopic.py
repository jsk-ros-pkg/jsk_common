#!/usr/bin/env python

import rospy
import rostopic


class ExtractSubtopic(object):

    def __init__(self):

        topic = rospy.resolve_name('~input')
        real_msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic)
        msg = rospy.wait_for_message(real_topic, real_msg_class)
        msg_subtopic = msg_eval(msg)
        sub_msg_class = type(msg_subtopic)
        self.msg_eval = msg_eval
        self.pub = rospy.Publisher('~output', sub_msg_class, queue_size=1)
        self.sub = rospy.Subscriber(real_topic, real_msg_class, self.callback)

    def callback(self, msg):

        msg_subtopic = self.msg_eval(msg)
        self.pub.publish(msg_subtopic)


if __name__ == '__main__':

    rospy.init_node('extract_subtopic')
    node = ExtractSubtopic()
    rospy.spin()
