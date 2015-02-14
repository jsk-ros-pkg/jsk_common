#!/usr/bin/env python

from jsk_network_tools.msg import FC2OCS, OCS2FC
from jsk_network_tools.silverhammer_util import *
from threading import Lock, Thread
from socket import *
from struct import Struct
import os
import rospy
import signal
import sys

import roslib
from roslib.message import get_message_class
from std_msgs.msg import Time

class SilverHammerLowspeedStreamer():
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCS")
        try:
            self.send_message = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.last_send_time = None
        self.last_input_received_time = None
        self.last_send_time_pub = rospy.Publisher("~last_send_time", Time)
        self.last_input_received_time_pub = rospy.Publisher(
            "~last_input_received_time", Time)
        self.to_port = rospy.get_param("~to_port", 1024)
        self.to_ip = rospy.get_param("~to_ip", "127.0.0.1")
        self.send_rate = rospy.get_param("~send_rate", 1)
        self.latest_message = None
        self.socket_client = socket(AF_INET, SOCK_DGRAM)
        self.send_format = msgToStructFormat(self.send_message())
        self.sub = rospy.Subscriber("~input", 
                                    self.send_message, self.messageCallback)
        self.send_timer = rospy.Timer(rospy.Duration(1 / self.send_rate),
                                      self.sendTimerCallback)
        self.update_time_pub_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                                 self.timeTimerCallback)
    def timeTimerCallback(self, event):
        with self.lock:
            if self.last_send_time:
                self.last_send_time_pub.publish(self.last_send_time)
            else:
                self.last_send_time_pub.publish(Time(data=rospy.Time(0)))
            if self.last_input_received_time:
                self.last_input_received_time_pub.publish(
                    self.last_input_received_time)
            else:
                self.last_input_received_time_pub.publish(
                    Time(data=rospy.Time(0)))
    def messageCallback(self, msg):
        with self.lock:
            self.latest_message = msg
            self.last_input_received_time = rospy.Time.now()
    def sendTimerCallback(self, event):
        with self.lock:
            if self.latest_message:
                rospy.logdebug("sending message")
                packed_data = packMessage(self.latest_message, self.send_format)
                self.socket_client.sendto(packed_data, (self.to_ip, self.to_port))
                self.latest_message = None
                self.last_send_time = rospy.Time.now()
            else:
                rospy.loginfo("no message is available")

if __name__ == "__main__":
    rospy.init_node("silverhammer_lowspeed_streamer")
    st = SilverHammerLowspeedStreamer()
    rospy.spin()



