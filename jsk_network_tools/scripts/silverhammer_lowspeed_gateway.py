#!/usr/bin/env python

from jsk_network_tools.msg import FC2OSC, OSC2FC
from jsk_network_tools.silverhammer_util import *
from threading import Lock, Thread
from socket import *
from struct import Struct
import os
import rospy
import signal
import sys

class SilverHammerUDPListener(Thread):
    def __init__(self, server, buffer_size, format, message, pub):
        super(SilverHammerUDPListener, self).__init__()
        self.server = server
        self.format = format
        self.pub = pub
        self.message = message
        self.buffer_size = buffer_size
    def run(self):
        while not rospy.is_shutdown():
            recv_data, addr = self.server.recvfrom(self.buffer_size)
            msg = unpackMessage(recv_data, self.format, self.message)
            self.pub.publish(msg)
            print "received:", msg

class SilverHammerLowspeedGateway():
    def __init__(self):
        self.is_fc = rospy.get_param("~is_fc", True)
        if self.is_fc:
            self.send_message = FC2OSC
            self.receive_message = OSC2FC
        else:
            self.send_message = OSC2FC
            self.receive_message = FC2OSC
        self.lock = Lock()
        self.to_port = rospy.get_param("~to_port", 1024)
        self.to_ip = rospy.get_param("~to_ip", "127.0.0.1")
        self.receive_port = rospy.get_param("~receive_port", 1025)
        self.receive_ip = rospy.get_param("~receive_ip", "127.0.0.1")
        self.receive_buffer = rospy.get_param("~receive_buffer_size", 250)
        self.send_rate = rospy.get_param("~send_rate", 1)
        self.socket_client = socket(AF_INET, SOCK_DGRAM)
        self.socket_server = socket(AF_INET, SOCK_DGRAM)
        self.socket_server.bind((self.receive_ip, self.receive_port))
        self.latest_message = None
        self.send_format = msgToStructFormat(self.send_message())
        self.receive_format = msgToStructFormat(self.receive_message())
        self.sub = rospy.Subscriber("~input", 
                                    self.send_message, self.messageCallback)
        self.pub = rospy.Publisher("~output", self.receive_message)
        self.send_timer = rospy.Timer(rospy.Duration(1 / self.send_rate),
                                      self.sendTimerCallback)
        self.receive_thread = SilverHammerUDPListener(self.socket_server, 
                                                      self.receive_buffer,
                                                      self.receive_format,
                                                      self.receive_message,
                                                      self.pub)
            
        self.receive_thread.start()
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
                

if __name__ == "__main__":
    rospy.init_node("silverhammer")
    gw = SilverHammerLowspeedGateway()
    rospy.spin()
    rospy.signal_shutdown("normal exit")
    os.kill(os.getpid(), signal.SIGKILL)

