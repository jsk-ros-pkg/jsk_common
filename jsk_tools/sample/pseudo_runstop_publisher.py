#!/usr/bin/env python

from distutils.version import LooseVersion as version

import pkg_resources
import rospy
import std_msgs.msg


class PseudoRunstopPublisher(object):

    def __init__(self):
        duration_time = rospy.get_param('~duration_time', 30)
        self.runstop = True
        self.pub = rospy.Publisher('~runstop', std_msgs.msg.Bool,
                                   queue_size=1)
        kwargs = dict(
            period=rospy.Duration(duration_time),
            callback=self.publish_callback,
            oneshot=False,
        )
        if version(pkg_resources.get_distribution('rospy').version) \
                >= version('1.12.0'):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            kwargs['reset'] = True
        self.timer = rospy.Timer(**kwargs)

    def publish_callback(self, event):
        if self.runstop is True:
            rospy.loginfo('Runstop is enabled.')
        else:
            rospy.loginfo('Runstop is disabled.')
        self.pub.publish(self.runstop)
        self.runstop = not self.runstop


if __name__ == '__main__':
    rospy.init_node('pseudo_runstop_publisher')
    pub = PseudoRunstopPublisher()
    rospy.spin()
