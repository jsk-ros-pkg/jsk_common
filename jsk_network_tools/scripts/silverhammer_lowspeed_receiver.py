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

class SilverHammerUDPListener():
    def __init__(self, server, buffer_size, format, message, pub):
        self.server = server
        self.format = format
        self.pub = pub
        self.message = message
        self.buffer_size = buffer_size
    def run(self):
        recv_data, addr = self.server.recvfrom(self.buffer_size)
        msg = unpackMessage(recv_data, self.format, self.message)
        self.pub.publish(msg)
        print "received:", msg

class SilverHammerLowspeedReceiver():
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCS")
        try:
            self.receive_message = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.receive_port = rospy.get_param("~receive_port", 1024)
        self.receive_ip = rospy.get_param("~receive_ip", "127.0.0.1")
        self.receive_buffer = rospy.get_param("~receive_buffer_size", 250)
        self.socket_server = socket(AF_INET, SOCK_DGRAM)
        self.socket_server.bind((self.receive_ip, self.receive_port))
        self.receive_format = msgToStructFormat(self.receive_message())
        self.pub = rospy.Publisher("~output", self.receive_message)
        self.last_received_time = None
        self.last_received_time_pub = rospy.Publisher(
            "~last_received_time", Time)
        self.last_publish_output_time = None
        self.last_publish_output_time_pub = rospy.Publisher(
            "~last_publish_output_time", Time)
        self.update_time_pub_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                                 self.timeTimerCallback)
    def timeTimerCallback(self, event):
        with self.lock:
            if self.last_received_time:
                self.last_received_time_pub.publish(
                    Time(data=self.last_received_time))
            else:
                self.last_received_time_pub.publish(
                    Time(data=rospy.Time(0)))
            if self.last_publish_output_time:
                self.last_publish_output_time_pub.publish(
                    Time(data=self.last_publish_output_time))
            else:
                self.last_publish_output_time_pub.publish(
                    Time(data=rospy.Time(0)))
    def run(self):
        while not rospy.is_shutdown():
            recv_data, addr = self.socket_server.recvfrom(self.receive_buffer)
            msg = unpackMessage(recv_data, self.receive_format, 
                                self.receive_message)
            with self.lock:
                self.last_received_time = rospy.Time.now()
            self.pub.publish(msg)
            with self.lock:
                self.last_publish_output_time = rospy.Time.now()
            rospy.logdebug("received:", msg)

if __name__ == "__main__":
    rospy.init_node("silverhammer_lowspeed_receiver")
    rc = SilverHammerLowspeedReceiver()
    rc.run()
