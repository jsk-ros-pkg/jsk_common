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
}


class BooleanNode(object):

    def __init__(self):
        self.pub = rospy.Publisher(
            '~output',
            std_msgs.msg.Bool, queue_size=1)
        self.boolean_operator = rospy.get_param('~operator')
        if self.boolean_operator.lower() not in OPERATORS:
            msg = '{} not supported.'.format(self.boolean_operator)
            msg += ' Supported operators are {}'.format(list(OPERATORS.keys()))
            rospy.loerr(msg)
            sys.exit(1)
        self.boolean_operator = OPERATORS[self.boolean_operator]

        self.n_input = rospy.get_param('~number_of_input', 2)
        self.n_input = int(self.n_input)
        if self.n_input <= 1:
            rospy.logerr('~number_of_input should be greater than 1.')
            sys.exit(1)

        self.data = {}
        self.subs = []
        for i in range(self.n_input):
            self.sub = rospy.Subscriber(
                '~input{}'.format(i + 1),
                std_msgs.msg.Bool,
                callback=lambda msg: self.callback('{}'.format(i + 1), msg),
                queue_size=1)

        rate = rospy.get_param('~rate', 100)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 100.')
            rate = 100
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def callback(self, topic_name, msg):
        self.data[topic_name] = msg.data

    def timer_cb(self, timer):
        if len(self.data) == 0:
            return
        flag = reduce(self.boolean_operator, self.data.values())
        self.pub.publish(
            std_msgs.msg.Bool(flag))


if __name__ == '__main__':
    rospy.init_node('boolean_node')
    node = BooleanNode()
    rospy.spin()
