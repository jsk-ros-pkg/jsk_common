#!/usr/bin/env python

import rospy
import roslib
import roslib.message
import sys
from roslib.message import get_message_class
from jsk_network_tools.silverhammer_util import *
from struct import calcsize
from StringIO import StringIO

def usage():
    print("silverhammer_lowspeed_check_size.py message_packege/Message")

def checkSize(class_str):
    try:
        message_class = get_message_class(class_str)
        format = msgToStructFormat(message_class)
        ros_msg = message_class()
        ros_buffer = StringIO()
        rospy.msg.serialize_message(ros_buffer, 0, ros_msg)
        print(class_str)
        print( "  binary format: ", format)
        print("  size:               ", calcsize(format), "bytes")
        print("                      ", 8 * calcsize(format), "bits")
        print("  size w/ UDP header: ", calcsize(format) + 36, "bytes")
        print("                      ", 8 * (calcsize(format) + 36), "bits")
        print("  ROS size:           ", ros_buffer.len, "bytes")
        print("                      ", ros_buffer.len * 8, "bits")
    except Exception as e:
        print("cannot serialize %s" % class_str)
        print("  error: ", e.message)
        return

    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        usage()
        sys.exit(1)
    checkSize(sys.argv[1])
