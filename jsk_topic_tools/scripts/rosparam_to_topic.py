#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String


class RosparamToTopic(object):

    def __init__(self):
        rosparam = rospy.get_param('~rosparam')
        if not isinstance(rosparam, dict):
            rosparam = {'param': rosparam}
        self.publishers = []
        for p_name, p_value in rosparam.items():
            topic_name = '~output/{}'.format(p_name)
            if isinstance(p_value, int):
                pub = rospy.Publisher(topic_name, Int32, queue_size=1)
            elif isinstance(p_value, float):
                pub = rospy.Publisher(topic_name, Float32, queue_size=1)
            elif isinstance(p_value, basestring):
                pub = rospy.Publisher(topic_name, String, queue_size=1)
            else:
                rospy.logwarn('Unsupported rosparam. name: {}, type: {}'
                            .format(p_name, type(p_value)))
                pub = None
            self.publishers.append(pub)
        rospy.Timer(rospy.Duration(0.1), self.cb, oneshot=False)

    def cb(self, event):
        rosparam = rospy.get_param('~rosparam')
        if not isinstance(rosparam, dict):
            rosparam = {'param': rosparam}
        for pub, p_value in zip(self.publishers, rosparam.values()):
            if pub is None:
                continue
            if isinstance(p_value, int):
                msg = Int32(data=p_value)
            elif isinstance(p_value, float):
                msg = Float32(data=p_value)
            elif isinstance(p_value, basestring):
                msg = String(data=p_value)
            else:
                raise ValueError
            pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('rosparam_to_topic')
    RosparamToTopic()
    rospy.spin()
