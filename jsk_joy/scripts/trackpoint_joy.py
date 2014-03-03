#!/usr/bin/env python

# publish ThinkPad trackpoint and button status as Joy.
# you need to execute following command beforehand.
# sudo chmod a+rw /dev/input/mice 

import rospy
import roslib
import struct

from sensor_msgs.msg import Joy


def main():
    rospy.sleep(1)
    rospy.init_node('trackpoint_joy')
    trackpoint_joy_pub = rospy.Publisher('/trackpoint/joy', Joy)
    tp_file = open( "/dev/input/mice", "rb" )

    while not rospy.is_shutdown():
        buf = tp_file.read(3)
        button = ord( buf[0] )
        bLeft = button & 0x1
        bMiddle = ( button & 0x4 ) > 0
        bRight = ( button & 0x2 ) > 0
        x,y = struct.unpack( "bb", buf[1:] )
        rospy.loginfo("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft, bMiddle, bRight, x, y) )
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.axes = [x, y]
        joy_msg.buttons = [bLeft, bMiddle, bRight]
        trackpoint_joy_pub.publish(joy_msg)
        # rospy.sleep(0.1)
    tp_file.close();


if __name__ == '__main__':
    main()
