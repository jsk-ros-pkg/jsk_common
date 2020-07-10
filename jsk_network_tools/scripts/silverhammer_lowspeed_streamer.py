#!/usr/bin/env python

from jsk_network_tools.msg import FC2OCS, OCS2FC
from jsk_network_tools.silverhammer_util import *
from threading import Lock, Thread
from socket import *
from struct import Struct
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
import os
import rospy
import signal
import sys

import roslib
from roslib.message import get_message_class
from std_msgs.msg import Time
from jsk_network_tools.srv import SetSendRate, SetSendRateResponse

class SilverHammerLowspeedStreamer():
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCS")
        try:
            self.send_message = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.launched_time = rospy.Time.now()
        self.diagnostic_updater = diagnostic_updater.Updater()
        self.diagnostic_updater.setHardwareID("none")
        self.diagnostic_updater.add("LowspeedStreamer", self.diagnosticCallback)
        self.send_num = 0
        self.last_send_time = rospy.Time(0)
        self.last_input_received_time = rospy.Time(0)
        self.last_send_time_pub = rospy.Publisher("~last_send_time", Time, queue_size=1)
        self.last_input_received_time_pub = rospy.Publisher(
            "~last_input_received_time", Time, queue_size=1)
        self.to_port = rospy.get_param("~to_port", 1024)
        self.to_ip = rospy.get_param("~to_ip", "127.0.0.1")
        self.send_rate = rospy.get_param("~send_rate", 1.0)
        self.event_driven = rospy.get_param("~event_driven", False)
        self.latest_message = None
        self.socket_client = socket(AF_INET, SOCK_DGRAM)
        self.send_format = msgToStructFormat(self.send_message())
        self.sub = rospy.Subscriber("~input", 
                                    self.send_message, self.messageCallback)
        if not self.event_driven:
            self.send_timer = rospy.Timer(rospy.Duration(1.0 / self.send_rate),
                                          self.sendTimerCallback)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                            self.diagnosticTimerCallback)
        self.send_rate_service = rospy.Service('~set_send_rate', SetSendRate, self.setSendRate)

    def setSendRate(self, req):
        try:
            if self.event_driven:
                rospy.logerr("failed to change send_rate. event_driven is enabled.")
                return SetSendRateResponse(ok=False)
            if self.send_timer.is_alive():
                self.send_timer.shutdown()
            self.send_rate = req.rate
            rospy.set_param("~send_rate", self.send_rate)
#            self.send_rate = rospy.get_param("~send_rate", 1.0)
            rospy.loginfo("send_rate is set to %f" % self.send_rate)
            self.send_timer = rospy.Timer(rospy.Duration(1.0 / self.send_rate),
                                          self.sendTimerCallback)
            return SetSendRateResponse(ok=True)
        except Exception as e:
            rospy.logerr("failed to set send_rate: %s" % e)
            return SetSendRateResponse(ok=False)

    def diagnosticTimerCallback(self, event):
        self.diagnostic_updater.update()
    def diagnosticCallback(self, stat):
        # always OK
        stat.summary(DiagnosticStatus.OK, "OK")
        with self.lock:
            now = rospy.Time.now()
            stat.add("Uptime [sec]",
                     (now - self.launched_time).to_sec())
            stat.add("Time from the last sending [sec]",
                     (now - self.last_send_time).to_sec())
            stat.add("Number of transmission", self.send_num)
            stat.add("Time from the last input [sec]",
                     (now - self.last_input_received_time).to_sec())
            # properties
            stat.add("UDP address", self.to_ip)
            stat.add("UDP port", self.to_port)
            stat.add("EventDrivenMode", self.event_driven)
            self.last_send_time_pub.publish(self.last_send_time)
            self.last_input_received_time_pub.publish(self.last_input_received_time)
        return stat
    def messageCallback(self, msg):
        with self.lock:
            self.latest_message = msg
            self.last_input_received_time = rospy.Time.now()
            if self.event_driven:
                self.sendMessage(msg)
    def sendMessage(self, msg):
        packed_data = packMessage(msg, self.send_format)
        self.socket_client.sendto(packed_data, (self.to_ip, self.to_port))
        self.last_send_time = rospy.Time.now()
        self.send_num = self.send_num + 1
    def sendTimerCallback(self, event):
        with self.lock:
            if self.latest_message:
                rospy.logdebug("sending message")
                self.sendMessage(self.latest_message)
                self.latest_message = None
            else:
                rospy.loginfo("no message is available")

if __name__ == "__main__":
    rospy.init_node("silverhammer_lowspeed_streamer")
    st = SilverHammerLowspeedStreamer()
    rospy.spin()



