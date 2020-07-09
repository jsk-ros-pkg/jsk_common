#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import AllTypeTest
import unittest

def defaultMessage():
    msg = AllTypeTest()
    msg.bool_array[0] = True
    msg.uint8_atom = 12
    msg.uint8_array = [ord(x) for x in 'abcd']
    msg.int8_atom = 12
    msg.int8_array[0] = 12
    msg.uint16_atom = 12
    msg.uint16_array[0] = 12
    msg.uint32_atom = 12
    msg.uint32_array[0] = 12
    msg.int32_atom = 12
    msg.int32_array[0] = 12
    msg.uint64_atom = 12
    msg.uint64_array[0] = 12
    msg.int64_atom = 12
    msg.int64_array[0] = 12
    msg.float32_atom = 12
    msg.float32_array[0] = 12
    msg.float64_atom = 12
    msg.float64_array[0] = 12
    return msg

def timerCallback(event):
    msg = defaultMessage()
    pub.publish(msg)
    
relayed_message = None
def messageCallback(msg):
    global relayed_message
    relayed_message = msg
        
class TestLowSpeed(unittest.TestCase):
    def test_topic_compare(self):
        global relayed_message
        # compare two messages
        reference = defaultMessage()
        self.assertTrue(relayed_message.bool_atom == reference.bool_atom)
        for i in range(len(relayed_message.bool_array)):
            self.assertTrue(relayed_message.bool_array[i] == reference.bool_array[i])
        self.assertTrue(relayed_message.uint8_atom == reference.uint8_atom)
        print(relayed_message.uint8_array)
        print(reference.uint8_array)
        self.assertTrue([ord(x) if isinstance(x, str) else x for x in relayed_message.uint8_array] == reference.uint8_array)
        self.assertTrue(relayed_message.int8_atom == reference.int8_atom)
        for i in range(len(relayed_message.int8_array)):
            self.assertTrue(relayed_message.int8_array[i] == reference.int8_array[i])
        self.assertTrue(relayed_message.uint16_atom == reference.uint16_atom)
        for i in range(len(relayed_message.uint16_array)):
            self.assertTrue(relayed_message.uint16_array[i] == reference.uint16_array[i])
        self.assertTrue(relayed_message.int32_atom == reference.int32_atom)
        for i in range(len(relayed_message.int32_array)):
            self.assertTrue(relayed_message.int32_array[i] == reference.int32_array[i])
        self.assertTrue(relayed_message.uint32_atom == reference.uint32_atom)
        for i in range(len(relayed_message.uint32_array)):
            self.assertTrue(relayed_message.uint32_array[i] == reference.uint32_array[i])
        self.assertTrue(relayed_message.int64_atom == reference.int64_atom)
        for i in range(len(relayed_message.int64_array)):
            self.assertTrue(relayed_message.int64_array[i] == reference.int64_array[i])
        self.assertTrue(relayed_message.uint64_atom == reference.uint64_atom)
        for i in range(len(relayed_message.uint64_array)):
            self.assertTrue(relayed_message.uint64_array[i] == reference.uint64_array[i])
        self.assertTrue(relayed_message.float32_atom == reference.float32_atom)
        for i in range(len(relayed_message.float32_array)):
            self.assertTrue(relayed_message.float32_array[i] == reference.float32_array[i])
        self.assertTrue(relayed_message.float64_atom == reference.float64_atom)
        for i in range(len(relayed_message.float64_array)):
            self.assertTrue(relayed_message.float64_array[i] == reference.float64_array[i])

    
if __name__ == "__main__":
    import rostest
    rospy.init_node("test_all_type_low_speed")
    pub = rospy.Publisher("original", AllTypeTest, queue_size=1)
    sub = rospy.Subscriber("relayed", AllTypeTest, messageCallback)
    rospy.Timer(rospy.Duration(0.5), timerCallback)
    rospy.loginfo("wait 10sec to acuumulate topics...")
    rospy.sleep(10)
    rostest.rosrun("jsk_network_tools", "test_low_speed", TestLowSpeed)


