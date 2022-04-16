#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import unittest

import rospy
import rostest

# load rostopic_host_sanity
__name__ == 'tmp'
file = os.path.join((os.path.dirname(os.path.abspath(__file__))), "../src/rostopic_host_sanity")
exec(open(file).read())
__name__ == '__main__'

NAME = 'test_rostopic_host_sanity'

class ROSTopicHostSanityTest(unittest.TestCase):
    def __init__(self, *args):
        super(ROSTopicHostSanityTest, self).__init__(*args)
        rospy.init_node(NAME)

    def test_rostopic_host_sanity(self):
        main() # call trostopic_host_sanity
        assert(True)

if __name__ == '__main__':
    try:
        rostest.run('jsk_tools', NAME, ROSTopicHostSanityTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting")
