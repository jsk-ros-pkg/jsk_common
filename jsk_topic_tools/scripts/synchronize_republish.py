#!/usr/bin/env python

import rospy
import message_filters

from rostopic import get_topic_class


def callback(*msgs):
    for msg, pub in zip(msgs, pubs):
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('synchrnoze_republish')
    topics = rospy.get_param('~topics')
    pubs = []
    subs = []
    for i, topic in enumerate(topics):
        topic = rospy.resolve_name(topic)
        msg_class = get_topic_class(topic, blocking=True)[0]
        pub = rospy.Publisher(
            '~pub_{0:0>2}'.format(i), msg_class, queue_size=1)
        pubs.append(pub)
        sub = message_filters.Subscriber(topic, msg_class)
        subs.append(sub)
    sync = message_filters.TimeSynchronizer(subs, queue_size=100)
    sync.registerCallback(callback)
    rospy.spin()
