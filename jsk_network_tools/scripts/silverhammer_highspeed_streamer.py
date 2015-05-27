#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import FC2OCSLargeData
from jsk_network_tools.silverhammer_util import *
from threading import Lock
from StringIO import StringIO
from io import BytesIO
from socket import *
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
import roslib
from roslib.message import get_message_class
from std_msgs.msg import Time
from dynamic_reconfigure.server import Server
from jsk_network_tools.cfg import SilverhammerHighspeedStreamerConfig

class SilverHammerStreamer:
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCSLargeData")
        try:
            self.message_class = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.dynamic_reconfigure = Server(SilverhammerHighspeedStreamerConfig, self.dynamicReconfigureCallback)
        self.launched_time = rospy.Time.now()
        self.packet_interval = None
        self.diagnostic_updater = diagnostic_updater.Updater()
        self.diagnostic_updater.setHardwareID("none")
        # register function to publish diagnostic
        self.diagnostic_updater.add("HighspeedStreamer", self.diagnosticCallback)
        self.send_port = rospy.get_param("~to_port", 16484)
        self.send_ip = rospy.get_param("~to_ip", "localhost")
        self.last_send_time = rospy.Time(0)
        self.last_input_received_time = rospy.Time(0)
        self.last_send_time_pub = rospy.Publisher("~last_send_time", Time)
        self.last_input_received_time_pub = rospy.Publisher(
            "~last_input_received_time", Time)
        self.send_num = 0
        self.rate = rospy.get_param("~send_rate", 2)   #2Hz
        self.socket_client = socket(AF_INET, SOCK_DGRAM)
        self.packet_size = rospy.get_param("~packet_size", 1400) # for MTU:=1500
        subscriber_info = subscribersFromMessage(self.message_class())
        self.messages = {}
        self.subscribe(subscriber_info)
        self.counter = 0
        self.send_timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                      self.sendTimerCallback)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                            self.diagnosticTimerCallback)
    def dynamicReconfigureCallback(self, config, level):
        with self.lock:
            self.bandwidth = config.bandwidth
            self.packet_sleep_sum = config.packet_sleep_sum
            return config
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
            stat.add("UDP address", self.send_ip)
            stat.add("UDP port", self.send_port)
            stat.add("Expected Hz to send", "%f Hz" % self.rate)
            stat.add("Packet size", "%d bytes" % self.packet_size)
            if self.packet_interval:
                stat.add("Latest nterval duration between packets",
                         self.packet_interval)
            self.last_send_time_pub.publish(self.last_send_time)
            self.last_input_received_time_pub.publish(self.last_input_received_time)
        return stat
    def diagnosticTimerCallback(self, event):
        self.diagnostic_updater.update()
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
    def sendTimerCallback(self, event):
        buffer = StringIO()
        with self.lock:
            msg = self.message_class()
            subscriber_info = subscribersFromMessage(msg)
            for topic, message_class in subscriber_info:
                field_name = topic[1:].replace("/", "__")
                if topic in self.messages:
                    setattr(msg, field_name, self.messages[topic])
            rospy.msg.serialize_message(buffer, 0, msg)
            self.last_send_time = rospy.Time.now()
        # send buffer as UDP
        self.sendBuffer(buffer.getvalue(), buffer.len * 8)
    def sendBuffer(self, buffer, buffer_size):
        self.send_num = self.send_num + 1
        packets = separateBufferIntoPackets(
            self.counter, buffer, self.packet_size)
        total_sec = float(buffer_size) / self.bandwidth
        packet_interval = total_sec / len(packets)
        self.packet_interval = packet_interval
        rospy.loginfo("sending %d packets", len(packets))
        rospy.loginfo("sending %d bits with %f interval" 
                      % (buffer_size, packet_interval))
        rospy.loginfo("total time to send is %f sec" % total_sec)
        r = rospy.Rate(1 / packet_interval / self.packet_sleep_sum)
        for p, i in zip(packets, range(len(packets))):
            self.socket_client.sendto(p.pack(), (self.send_ip, self.send_port))
            if i % self.packet_sleep_sum == 0:
                r.sleep()
        self.counter = self.counter + 1
        if self.counter > 65535:
            self.counter = 0
    def subscribe(self, subscriber_infos):
        self.subscribers = []
        for topic, message_class in subscriber_infos:
            sub = rospy.Subscriber(topic, message_class, 
                                   self.genMessageCallback(topic))
            self.subscribers.append(sub)
    def messageCallback(self, msg, topic):
        with self.lock:
            self.messages[topic] = msg
            self.last_input_received_time = rospy.Time.now()
    def genMessageCallback(self, topic):
        return lambda msg: self.messageCallback(msg, topic)

if __name__ == "__main__":
    rospy.init_node("silverhammer_highspeed_streamer")
    streamer = SilverHammerStreamer()
    rospy.spin()
    
