#!/usr/bin/env python

import rospy
import message_filters

from rostopic import get_topic_class

def callback(*msgs):
    rospy.loginfo("synchronized")
    for msg, pub in zip(msgs, publishers):
        pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("synchrnoze_republish")
    topics = rospy.get_param("~topics")
    # estimate message type from topics
    types = [get_topic_class(rospy.resolve_name(tp), blocking=True) for tp in topics]
    # publishers
    publishers = [rospy.Publisher("~pub_{0:0>2}".format(i), tp[0]) for tp, i in zip(types, range(len(types)))]
    subs = [message_filters.Subscriber(tp[1], tp[0])
            for tp, i in zip(types, range(len(types)))]
    sync = message_filters.TimeSynchronizer(subs, queue_size=100)
    sync.registerCallback(callback)
    rospy.spin()
    
