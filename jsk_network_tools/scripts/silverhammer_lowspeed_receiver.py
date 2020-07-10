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
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

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
        print("received:", msg)

class SilverHammerLowspeedReceiver():
    def __init__(self):
        message_class_str = rospy.get_param("~message",
                                            "jsk_network_tools/FC2OCS")
        try:
            self.receive_message = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.launched_time = rospy.Time.now()
        self.diagnostic_updater = diagnostic_updater.Updater()
        self.diagnostic_updater.setHardwareID("none")
        self.diagnostic_updater.add("LowspeedReceiver", self.diagnosticCallback)
        self.received_num = 0
        self.receive_port = rospy.get_param("~receive_port", 1024)
        self.receive_ip = rospy.get_param("~receive_ip", "127.0.0.1")
        self.receive_buffer = rospy.get_param("~receive_buffer_size", 250)
        self.socket_server = socket(AF_INET, SOCK_DGRAM)
        self.socket_server.settimeout(None)
        self.socket_server.bind((self.receive_ip, self.receive_port))
        self.receive_format = msgToStructFormat(self.receive_message())
        self.pub = rospy.Publisher("~output", self.receive_message, queue_size=1)
        self.last_received_time = rospy.Time(0)
        self.last_received_time_pub = rospy.Publisher(
            "~last_received_time", Time, queue_size=1)
        self.last_publish_output_time = rospy.Time(0)
        self.last_publish_output_time_pub = rospy.Publisher(
            "~last_publish_output_time", Time, queue_size=1)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                            self.diagnosticTimerCallback)
    def diagnosticTimerCallback(self, event):
        self.diagnostic_updater.update()
        # and publish time
        with self.lock:
            self.last_publish_output_time_pub.publish(self.last_publish_output_time)
            self.last_received_time_pub.publish(self.last_received_time)
    def diagnosticCallback(self, stat):
        # always OK
        stat.summary(DiagnosticStatus.OK, "OK")
        with self.lock:
            now = rospy.Time.now()
            stat.add("Uptime [sec]",
                     (now - self.launched_time).to_sec())
            stat.add("Time from the last reception [sec]",
                     (now - self.last_received_time).to_sec())
            stat.add("Time from the last publish ~output [sec]",
                     (now - self.last_publish_output_time).to_sec())
            stat.add("UDP address", self.receive_ip)
            stat.add("UDP port", self.receive_port)
        return stat
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
