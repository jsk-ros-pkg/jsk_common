#!/usr/bin/env python

from __future__ import print_function

import sys
import unittest

import rospy

from std_srvs.srv import Trigger


PKG = 'jsk_data'
NAME = 'test_data_collection_server'

class TestDataCollectionServer(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def _check(self, no_logging=False):
        sys.stdout = f = StringIO.StringIO()
        self.logging_throttle(id='a', logging_func=print, period=3, msg='spam')
        sys.stdout = sys.__stdout__
        if no_logging:
            self.assertFalse(f.getvalue(), 'spam\n')
        else:
            self.assertEqual(f.getvalue(), 'spam\n')

    def test_requests(self):
        rospy.wait_for_service('/data_collection_server_request/save_request')
        save_request = rospy.ServiceProxy('/data_collection_server_request/save_request', Trigger)
        ret = save_request()
        self.assertTrue(ret.success)

    def test_timers(self):
        rospy.wait_for_service('/data_collection_server_timer/start_request')
        start_request = rospy.ServiceProxy('/data_collection_server_timer/start_request', Trigger)
        ret = start_request()
        self.assertTrue(ret.success)
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDataCollectionServer)

