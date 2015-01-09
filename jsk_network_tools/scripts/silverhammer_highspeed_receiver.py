#!/usr/bin/env python

import rospy
from jsk_network_tools.msg import FC2OCSLargeData
from jsk_network_tools.silverhammer_util import *
from threading import Lock
from StringIO import StringIO
from io import BytesIO
from socket import *
from struct import pack

class SilverHammerReceiver:
    def __init__(self):
        self.lock = Lock()
        self.receive_port = rospy.get_param("~receive_port", 16484)
        self.receive_ip = rospy.get_param("~receive_ip", "localhost")
        self.topic_prefix = rospy.get_param("~topic_prefix", "/from_fc")
        self.publishers = publishersFromMessage(FC2OCSLargeData, self.topic_prefix)
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
                self.concatenatePackets()
            elif packet.seq_id != self.packets[-1].seq_id:
                rospy.logerr("packet lossed!")
                self.packets = [packet]   #reset packet
    def concatenatePackets(self):
        if self.packets[0].id == 0:
            seq_id = self.packets[0].seq_id
            # all the packet has same seq_id
            if len([p for p in self.packets if p.seq_id == seq_id]) == len(self.packets):
                # received in order
                if [p.id for p in self.packets] == range(self.packets[0].num):
                    b = StringIO()
                    for p in self.packets:
                        b.write(p.data)
                    deserialized_data = []
                    rospy.msg.deserialize_messages(b, deserialized_data,
                                                   FC2OCSLargeData)
                    rospy.loginfo("received %d message" % len(deserialized_data))
                    if len(deserialized_data) > 0:
                        # publish data
                        msg = deserialized_data[0]
                        messages = decomposeLargeMessage(msg, self.topic_prefix)
                        for pub in self.publishers:
                            if pub.name in messages:
                                pub.publish(messages[pub.name])
                        #self.pub.publish(deserialized_data[0])
                        
                else:
                    rospy.logerr("order is not correct")
            else:
                rospy.logerr("missed some packets")
        else:
            rospy.logerr("failed to receive first packet")
        self.packets = []


if __name__ == "__main__":
    rospy.init_node("silverhammer_highspeed_receiver")
    receiver = SilverHammerReceiver()
    receiver.run()
    
