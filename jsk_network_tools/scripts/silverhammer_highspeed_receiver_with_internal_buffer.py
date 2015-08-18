#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import FC2OCSLargeData, SilverhammerInternalBuffer
from jsk_network_tools.silverhammer_util import *
from threading import Lock
from StringIO import StringIO
from std_msgs.msg import Time
from io import BytesIO
from socket import *
from struct import pack
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
import roslib
from roslib.message import get_message_class

#ifdef USE_THREAD
# from threading import Thread
# from Queue import Queue
#else
from multiprocessing import Process, Queue
#endif

class SilverHammerReceiver:
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCSLargeData")
        try:
            self.message_class = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.diagnostic_updater = diagnostic_updater.Updater()
        self.diagnostic_updater.setHardwareID("none")
        self.diagnostic_updater.add("HighspeedReceiver", self.diagnosticCallback)
        self.latch = rospy.get_param("~latch", True)
        self.pesimistic = rospy.get_param("~pesimistic", False)
        self.fragment_packets_torelance = rospy.get_param("~fragment_packets_torelance", 20)
        self.timestamp_overwrite_topics = rospy.get_param("~timestamp_overwrite_topics", [])
        self.publish_only_if_updated_topics = rospy.get_param("~publish_only_if_updated_topics", [])
        self.prev_seq_ids = {}
        self.receive_port = rospy.get_param("~receive_port", 16484)
        self.receive_ip = rospy.get_param("~receive_ip", "localhost")
        self.topic_prefix = rospy.get_param("~topic_prefix", "/from_fc")
        if not self.topic_prefix.startswith("/"):
            self.topic_prefix = "/" + self.topic_prefix
        if self.topic_prefix == "/":
            self.topic_prefix = ""
        self.publishers = publishersFromMessage(self.message_class,
                                                self.topic_prefix, 
                                                latch=self.latch)
        self.packet_size = rospy.get_param("~packet_size", 1400)   #2Hz
        self.launched_time = rospy.Time.now()
        self.last_received_time = rospy.Time(0)
        self.last_received_time_pub = rospy.Publisher("~last_received_time", Time)
        self.last_published_seq_id = -1
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0 / 10),
                                            self.diagnosticTimerCallback)
        self.packets_queue = Queue()
    def diagnosticCallback(self, stat):
        # always OK
        stat.summary(DiagnosticStatus.OK, "OK")
        with self.lock:
            now = rospy.Time.now()
            stat.add("Uptime [sec]",
                     (now - self.launched_time).to_sec())
            stat.add("Time from last input [sec]", 
                     (now - self.last_received_time).to_sec())
            stat.add("UDP address", self.receive_ip)
            stat.add("UDP port", self.receive_port)
        return stat
    def diagnosticTimerCallback(self, event):
        self.diagnostic_updater.update()
        with self.lock:
            self.last_received_time_pub.publish(self.last_received_time)
    def run(self):
        sub = rospy.Subscriber("~input", SilverhammerInternalBuffer, self.callback)
        rospy.spin()
    def callback(self, msg):
        b = StringIO()
        b.write(msg.data)
        deserialized_data = []
        rospy.loginfo("data size: %d" % len(msg.data))
        rospy.msg.deserialize_messages(b, deserialized_data,
                                       self.message_class)
        rospy.loginfo("received %d message" % len(deserialized_data))
        
        if len(deserialized_data) > 0:
            # publish data
            msg = deserialized_data[0]
            messages = decomposeLargeMessage(msg, self.topic_prefix)
            now = rospy.Time.now()
            for pub in self.publishers:
                if pub.name in messages:
                    if not pub.name in self.publish_only_if_updated_topics:
                        rospy.loginfo("publishing %s" % pub.name)
                        if pub.name in self.timestamp_overwrite_topics:
                            if (hasattr(messages[pub.name], "header") and
                                hasattr(messages[pub.name].header, "stamp")):
                                messages[pub.name].header.stamp = now
                        pub.publish(messages[pub.name])
                    #pub.publish(messages[pub.name])
                else:
                    rospy.logwarn("""cannot find '%s' in deserialized messages %s""" % (pub.name, messages.keys()))
            synchronized_publishers = []
            at_lest_one_topic = False
            # Check if there is any topic to update
            for pub in self.publishers:
                if pub.name in messages:
                    if pub.name in self.publish_only_if_updated_topics:
                        synchronized_publishers.append(pub)
                        if (hasattr(messages[pub.name], "header") and
                            hasattr(messages[pub.name].header, "stamp")):
                            messages[pub.name].header.stamp = now # Overwrite with the same timestamp
                        if (hasattr(messages[pub.name], "header") and
                            hasattr(messages[pub.name].header, "seq")):
                            # Skip rule
                            if (pub.name in self.prev_seq_ids.keys() and 
                                messages[pub.name].header.seq == self.prev_seq_ids[pub.name]):
                                rospy.loginfo("skip publishing %s " % (pub.name))
                                pass
                            else:
                                rospy.loginfo("messages[%s].header.seq: %d"% (pub.name,
                                                                              messages[pub.name].header.seq))
                                rospy.loginfo("self.prev_seq_ids[%s]: %d"  % (pub.name,
                                                                              messages[pub.name].header.seq))
                                self.prev_seq_ids[pub.name] = messages[pub.name].header.seq
                                at_lest_one_topic = True
            if at_lest_one_topic:
                for pub in synchronized_publishers:
                    pub.publish(messages[pub.name])
        else:
            rospy.logerr("missed some packets")


if __name__ == "__main__":
    rospy.init_node("silverhammer_highspeed_receiver")
    receiver = SilverHammerReceiver()
    receiver.run()
