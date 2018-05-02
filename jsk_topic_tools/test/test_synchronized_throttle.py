#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import unittest
from geometry_msgs.msg import PoseStamped
import message_filters as MF


class TestSynchronizedThrottle(unittest.TestCase):
    def setUp(self):
        self.pub = rospy.Publisher("/baz", PoseStamped, queue_size=1)
        rospy.sleep(1)
        self.subs = []
        self.start = rospy.Time(0)
        self.duration = rospy.Duration(5)
        self.count = 0
        self.max_diff = 100000
        self.timeout = rospy.Duration(10)
        self.finished = False

    def tearDown(self):
        for s in self.subs:
            s.unregister()

    def throttle_cb(self, *msgs):
        now = rospy.Time.now()
        if self.count == 0:
            self.start = now
        if now - self.start > self.duration:
            self.finished = True
        if not self.finished:
            self.count += 1
            msgs = sorted(list(msgs), key=lambda m: m.header.stamp)
            diff = msgs[-1].header.stamp - msgs[0].header.stamp
            self.max_diff = diff.to_sec()

    def wait(self):
        start = rospy.Time.now()
        while True:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                return False
            if self.finished:
                return True
            if rospy.Time.now() - start > self.timeout:
                return False

    def sync_baz_cb(self, msg):
        rospy.logdebug("baz")
        self.pub.publish(msg)

    def delay_baz_cb(self, msg):
        rospy.logdebug("delay")
        msg.header.stamp = rospy.Time.from_sec(msg.header.stamp.to_sec() - 0.001)
        self.pub.publish(msg)

    def test_sync(self):
        sub = rospy.Subscriber("/foo", PoseStamped, self.sync_baz_cb)
        subs = [
            MF.Subscriber("/foo/sync", PoseStamped, queue_size=1),
            MF.Subscriber("/bar/sync", PoseStamped, queue_size=1),
            MF.Subscriber("/baz/sync", PoseStamped, queue_size=1),
        ]
        sync = MF.TimeSynchronizer(subs, 5)
        sync.registerCallback(self.throttle_cb)

        self.subs += [sub]
        self.subs += subs

        self.assertTrue(self.wait(), "Wait for throttled topic")
        self.assertAlmostEqual(self.count, 5, delta=1)
        self.assertEqual(self.max_diff, 0.0)

    def test_sync_delay(self):
        self.subs += [
            rospy.Subscriber("/foo", PoseStamped, self.delay_baz_cb),
            rospy.Subscriber("/baz/sync", PoseStamped, self.throttle_cb),
        ]
        self.assertFalse(self.wait(), "Wait for throttled topic")

    def test_async(self):
        sub = rospy.Subscriber("/foo", PoseStamped, self.sync_baz_cb)
        subs = [
            MF.Subscriber("/foo/async", PoseStamped, queue_size=1),
            MF.Subscriber("/bar/async", PoseStamped, queue_size=1),
            MF.Subscriber("/baz/async", PoseStamped, queue_size=1),
        ]
        sync = MF.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.01)
        sync.registerCallback(self.throttle_cb)

        self.subs += [sub]
        self.subs += subs

        self.assertTrue(self.wait(), "Wait for throttled topic")
        self.assertAlmostEqual(self.count, 5, delta=1)
        self.assertEqual(self.max_diff, 0.0)

    def test_async_delay(self):
        sub = rospy.Subscriber("/foo", PoseStamped, self.delay_baz_cb)
        subs = [
            MF.Subscriber("/foo/async", PoseStamped, queue_size=1),
            MF.Subscriber("/bar/async", PoseStamped, queue_size=1),
            MF.Subscriber("/baz/async", PoseStamped, queue_size=1),
        ]
        sync = MF.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.01)
        sync.registerCallback(self.throttle_cb)

        self.subs += [sub]
        self.subs += subs

        self.assertTrue(self.wait(), "Wait for throttled topic")
        self.assertAlmostEqual(self.count, 5, delta=1)
        self.assertLess(self.max_diff, 0.002)


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_synchronized_throttle")
    rostest.rosrun("jsk_topic_tools", "test_synchronized_throttle", TestSynchronizedThrottle)
