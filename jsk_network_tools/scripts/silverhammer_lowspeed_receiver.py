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
        self.receive_port = rospy.get_param("~receive_port", 1024)
        self.receive_ip = rospy.get_param("~receive_ip", "127.0.0.1")
        self.receive_buffer = rospy.get_param("~receive_buffer_size", 250)
        self.socket_server = socket(AF_INET, SOCK_DGRAM)
        self.socket_server.bind((self.receive_ip, self.receive_port))
        self.receive_format = msgToStructFormat(self.receive_message())
        self.pub = rospy.Publisher("~output", self.receive_message)
    def messageCallback(self, msg):
        with self.lock:
            self.latest_message = msg
    def sendTimerCallback(self, event):
        with self.lock:
            if self.latest_message:
                rospy.loginfo("sending message")
                packed_data = packMessage(self.latest_message, self.send_format)
                self.socket_client.sendto(packed_data, (self.to_ip, self.to_port))
                self.latest_message = None
            else:
                rospy.loginfo("no message is available")
    def run(self):
        while not rospy.is_shutdown():
            recv_data, addr = self.socket_server.recvfrom(self.receive_buffer)
            msg = unpackMessage(recv_data, self.receive_format, 
                                self.receive_message)
            self.pub.publish(msg)
            print "received:", msg

if __name__ == "__main__":
    rospy.init_node("silverhammer_lowspeed_receiver")
    rc = SilverHammerLowspeedReceiver()
    rc.run()
