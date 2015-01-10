#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import FC2OCSLargeData
from jsk_network_tools.silverhammer_util import *
from threading import Lock
from StringIO import StringIO
from io import BytesIO
from socket import *

import roslib
from roslib.message import get_message_class

class SilverHammerStreamer:
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCSLargeData")
        try:
            self.message_class = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.send_port = rospy.get_param("~to_port", 16484)
        self.send_ip = rospy.get_param("~to_ip", "localhost")
        self.rate = rospy.get_param("~send_rate", 2)   #2Hz
        self.socket_client = socket(AF_INET, SOCK_DGRAM)
        self.packet_size = rospy.get_param("~packet_size", 1000)   #2Hz
        subscriber_info = subscribersFromMessage(self.message_class())
        self.messages = {}
        self.subscribe(subscriber_info)
        self.counter = 0
        self.send_timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                      self.sendTimerCallback)
        
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
        # send buffer as UDP
        self.sendBuffer(buffer.getvalue())
    def sendBuffer(self, buffer):
        packets = separateBufferIntoPackets(self.counter, buffer, self.packet_size)
        rospy.loginfo("sending %d packets", len(packets))
        for p in packets:
            self.socket_client.sendto(p.pack(), (self.send_ip, self.send_port))
        self.counter = self.counter + 1
        if self.counter > 65535:
            self.counter = 0
    def subscribe(self, subscriber_infos):
        self.subscribers = []
        for topic, message_class in subscriber_infos:
            sub = rospy.Subscriber(topic, message_class, self.genMessageCallback(topic))
            self.subscribers.append(sub)
    def messageCallback(self, msg, topic):
        with self.lock:
            self.messages[topic] = msg
    def genMessageCallback(self, topic):
        return lambda msg: self.messageCallback(msg, topic)

if __name__ == "__main__":
    rospy.init_node("silverhammer_highspeed_streamer")
    streamer = SilverHammerStreamer()
    rospy.spin()
    
