#!/usr/bin/env python
# -*- coding: utf-8 -*-

from importlib import import_module

import roslib.message
import rospy


def expr_eval(expr, modules):
    def eval_fn(topic, m, t):
        return eval(expr, modules, {'m': m})
    return eval_fn


class FilteredRelay(object):

    def __init__(self):
        import_list = rospy.get_param('~import', [])
        self.modules = {}
        for module in import_list:
            try:
                mod = import_module(module)
            except ImportError:
                rospy.logerr('Failed to import module: %s' % module)
            else:
                self.modules[module] = mod

        self.filter_expression = rospy.get_param('~filter', None)
        if self.filter_expression is not None:
            self.filter_expression = expr_eval(
                self.filter_expression, self.modules)

        self.output_type = rospy.get_param('~output_type')
        self.output_class = roslib.message.get_message_class(self.output_type)

        self.transform_expression = rospy.get_param('~transform', 'm')
        self.pub_transform = rospy.Publisher(
            '~output',
            self.output_class, queue_size=1)

        self.topic_name = rospy.resolve_name('~input')
        self.sub = rospy.Subscriber(
            self.topic_name,
            rospy.AnyMsg,
            callback=lambda msg, tn=self.topic_name: self.callback(tn, msg),
            queue_size=1)

    def publish(self, m):
        if self.output_type == 'std_msgs/Empty':
            self.pub_transform.publish()
            return
        if self.transform_expression is not None:
            try:
                res = eval(self.transform_expression, self.modules, {'m': m})
            except NameError as e:
                rospy.logerr("Expression using variables other than 'm': %s"
                             % e.message)
            except UnboundLocalError as e:
                rospy.logerr('Wrong expression:%s' % e.message)
            except Exception as e:
                rospy.logerr(str(e))
            else:
                if not isinstance(res, (list, tuple)):
                    res = [res]
                self.pub_transform.publish(*res)
        else:
            self.pub_transform.publish(m)

    def callback(self, topic_name, msg):
        if isinstance(msg, rospy.msg.AnyMsg):
            package, msg_type = msg._connection_header['type'].split('/')
            ros_pkg = package + '.msg'
            msg_class = getattr(import_module(ros_pkg), msg_type)
            self.sub.unregister()
            deserialized_sub = rospy.Subscriber(
                topic_name, msg_class,
                lambda msg, tn=topic_name: self.callback(tn, msg))
            self.sub = deserialized_sub
            msg = msg_class().deserialize(msg._buff)
        if self.filter_expression is not None:
            if self.filter_expression(topic_name, msg, rospy.Time.now()):
                self.publish(msg)
        else:
            self.publish(msg)


if __name__ == '__main__':
    rospy.init_node('filtered_relay')
    node = FilteredRelay()
    rospy.spin()
