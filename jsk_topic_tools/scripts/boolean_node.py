#!/usr/bin/env python

import sys
from operator import xor
from functools import reduce

import rospy
import std_msgs.msg


OPERATORS = {
    'or': lambda a, b: a or b,
    'and': lambda a, b: a and b,
    'xor': xor,
    'not': None,
}


class BooleanNode(object):

    def __init__(self):
        self.n_input = rospy.get_param('~number_of_input', 2)
        self.n_input = int(self.n_input)
        if self.n_input <= 0:
            rospy.logerr('~number_of_input should be greater than 0.')
            sys.exit(1)

        self.pubs = {}
        if self.n_input == 1:
            self.pubs['not'] = rospy.Publisher(
                '~output/not',
                std_msgs.msg.Bool, queue_size=1)
        else:
            for op_str in OPERATORS:
                self.pubs[op_str] = rospy.Publisher(
                    '~output/{}'.format(op_str),
                    std_msgs.msg.Bool, queue_size=1)

        self.data = {}
        self.subs = {}
        for i in range(self.n_input):
            topic_name = '~input{}'.format(i + 1)
            topic_name = rospy.resolve_name(topic_name)
            sub = rospy.Subscriber(
                topic_name,
                std_msgs.msg.Bool,
                callback=lambda msg, tn=topic_name: self.callback(tn, msg),
                queue_size=1)
            self.subs[topic_name] = sub

        rate = rospy.get_param('~rate', 100)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 100.')
            rate = 100
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def callback(self, topic_name, msg):
        self.data[topic_name] = msg.data

    def timer_cb(self, timer):
        if len(self.data) != self.n_input:
            return
        for op_str, pub in self.pubs.items():
            if op_str == 'not':
                flag = not list(self.data.values())[0]
            else:
                flag = reduce(OPERATORS[op_str], self.data.values())
            pub.publish(std_msgs.msg.Bool(flag))


if __name__ == '__main__':
    rospy.init_node('boolean_node')
    node = BooleanNode()
    rospy.spin()
