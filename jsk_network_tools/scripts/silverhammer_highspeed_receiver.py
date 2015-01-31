#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import FC2OCSLargeData
from jsk_network_tools.silverhammer_util import *
from threading import Lock
from StringIO import StringIO
from io import BytesIO
from socket import *
from struct import pack

import roslib
from roslib.message import get_message_class

class SilverHammerReceiver:
    def __init__(self):
        message_class_str = rospy.get_param("~message", 
                                            "jsk_network_tools/FC2OCSLargeData")
        try:
            self.message_class = get_message_class(message_class_str)
        except:
            raise Exception("invalid topic type: %s"%message_class_str)
        self.lock = Lock()
        self.pesimistic = rospy.get_param("~pesimistic", False)
        self.receive_port = rospy.get_param("~receive_port", 16484)
        self.receive_ip = rospy.get_param("~receive_ip", "localhost")
        self.topic_prefix = rospy.get_param("~topic_prefix", "/from_fc")
        if not self.topic_prefix.startswith("/"):
            self.topic_prefix = "/" + self.topic_prefix
        if self.topic_prefix == "/":
            self.topic_prefix = ""
        self.publishers = publishersFromMessage(self.message_class, self.topic_prefix)
        self.socket_server = socket(AF_INET, SOCK_DGRAM)
        self.socket_server.bind((self.receive_ip, self.receive_port))
        self.packet_size = rospy.get_param("~packet_size", 1000)   #2Hz
        self.packets = []
    def run(self):
        while not rospy.is_shutdown():
            recv_data, addr = self.socket_server.recvfrom(self.packet_size)
            packet = LargeDataUDPPacket.fromData(recv_data, self.packet_size)
            self.packets.append(packet)
            if packet.num - 1 == packet.id:
                # the end of packet
                try:
                    self.concatenatePackets()
                except Exception,e:
                    rospy.logerr("failed to concatenate packets: %s", e.message)
                finally:
                    self.packets = []
            elif packet.seq_id != self.packets[-1].seq_id:
                rospy.logerr("packet lossed!")
                self.packets = [packet]   #reset packet
    def concatenatePackets(self):
        if self.packets[0].id == 0:
            seq_id = self.packets[0].seq_id
            # all the packet has same seq_id
            if len([p for p in self.packets if p.seq_id == seq_id]) == len(self.packets):
                # received in order
                # sort order
                self.packets.sort(key=lambda p: p.id)
                packet_data_length = len(self.packets[0].data)
                packet_index = 0
                b = StringIO()
                if self.packets[0].num != len(self.packets):
                    rospy.logwarn("%d packet is missed", self.packets[0].num - len(self.packets))
                    if self.pesimistic:
                        rospy.logerr("pesimistic mode, give up to reconstruct message")
                        return
                for i in range(self.packets[0].num):
                    if self.packets[packet_index].id == i:
                        packet = self.packets[packet_index]
                        b.write(packet.data)
                        packet_index = packet_index + 1
                    else:
                        # fill by dummy data
                        b.write(chr(0) * packet_data_length)
                deserialized_data = []
                rospy.msg.deserialize_messages(b, deserialized_data,
                                               self.message_class)
                rospy.loginfo("received %d message" % len(deserialized_data))
                if len(deserialized_data) > 0:
                    # publish data
                    msg = deserialized_data[0]
                    messages = decomposeLargeMessage(msg, self.topic_prefix)
                    for pub in self.publishers:
                        if messages.has_key(pub.name):
                            rospy.loginfo("publishing %s" % pub.name)
                            pub.publish(messages[pub.name])
                        else:
                            rospy.logwarn("""cannot find '%s' in deserialized messages %s""" % (pub.name, messages.keys()))
                # else:
                #     rospy.logerr("order is not correct")
                #     rospy.logerr("collected packets: %d, expected packets: %d", len(self.packets), self.packets[0].num)
            else:
                rospy.logerr("missed some packets")
        else:
            rospy.logerr("failed to receive first packet")
        


if __name__ == "__main__":
    rospy.init_node("silverhammer_highspeed_receiver")
    receiver = SilverHammerReceiver()
    receiver.run()
    
