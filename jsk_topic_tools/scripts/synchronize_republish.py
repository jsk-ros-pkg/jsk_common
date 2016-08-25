#!/usr/bin/env python

import rospy
import message_filters

from rostopic import get_topic_class


def callback(*msgs):
    stamp = None
    for msg, pub in zip(msgs, pubs):
        if stamp is None:
            stamp = msg.header.stamp
        else:
            msg.header.stamp = stamp
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('synchrnoze_republish')
    topics = rospy.get_param('~topics')
    use_async = rospy.get_param('~approximate_sync', False)
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
    if use_async:
        sync = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=100, slop=0.1)
    else:
        sync = message_filters.TimeSynchronizer(subs, queue_size=100)
    sync.registerCallback(callback)
    rospy.spin()
