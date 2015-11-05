#!/usr/bin/env python

import unittest

import rospy

try:
    from std_msgs.msg import String
except:
    import roslib
    roslib.load_manifest("jsk_topic_tools")
    from std_msgs.msg import String


PKG = 'jsk_topic_tools'
NAME = 'test_block'


class TestBlock(unittest.TestCase):
    input_msg = None
    subscriber = None

    def __init__(self, *args):
        super(TestBlock, self).__init__(*args)
        rospy.init_node(NAME)
        self.output_original_pub = rospy.Publisher(
            'output_original', String, queue_size=1)
        self.input_original_sub = rospy.Subscriber(
            'input_original', String, self.original_topic_cb)

    def original_topic_cb(self, msg):
        self.running = True
        self.output_original_pub.publish(msg)

    def reset_subscriber(self):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None
        self.input_msg = None
        self.running = False

    def test_subscribe(self):
        self.reset_subscriber()
        self.subscriber = rospy.Subscriber("output", String, self.cb)
        rospy.loginfo("wait 10 seconds...")
        rospy.sleep(10)
        self.assertFalse(self.input_msg is None)

    def cb(self, msg):
        self.input_msg = msg

    def test_no_subscribe(self):
        self.reset_subscriber()
        # do not subscribe
        rospy.loginfo("wait 10 seconds...")
        rospy.sleep(10)
        self.assertTrue(self.input_msg is None)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestBlock)
