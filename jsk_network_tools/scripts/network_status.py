#!/usr/bin/python
import re
import rospy
from std_msgs.msg import Float32
from collections import deque
import os

def rosSafeName(name):
    return name.lower().replace("-", "_")

class NetworkStatus():
    def __init__(self):
        rospy.init_node('network')
        self.match_face = re.compile('(.+):(.*)')
        self.hz = rospy.get_param('~hz', 10)
        rospy.logdebug('publish network status (bps) at ' +  str(self.hz) + 'Hz')
        rospy.logdebug('usage:\n$rosrun jsk_network_tools network_status.py _hz:=[Hz]')
        self.init_publisher()
        rospy.Timer(rospy.Duration(1.0/self.hz), self.publish)
        rospy.spin()
    def init_publisher(self):
        faces = self.read_net_file()
        self.faces_map = {}
        self.non_local_transmit_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/transmit', Float32, queue_size=1)
        self.non_local_receive_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/receive', Float32, queue_size=1)
        self.non_local_transmit_kbps_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/transmit_kbps', Float32, queue_size=1)
        self.non_local_receive_kbps_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/receive_kbps', Float32, queue_size=1)
        self.non_local_transmit_mbps_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/transmit_mbps', Float32, queue_size=1)
        self.non_local_receive_mbps_pub = rospy.Publisher(rosSafeName(os.uname()[1]) + "/nonlocal" + '/receive_mbps', Float32, queue_size=1)
        for face in faces:
            pub_transmit = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/transmit', Float32, queue_size=1)
            pub_receive = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/receive', Float32, queue_size=1)
            pub_transmit_kbps = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/transmit_kbps', Float32, queue_size=1)
            pub_receive_kbps = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/receive_kbps', Float32, queue_size=1)
            pub_transmit_mbps = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/transmit_mbps', Float32, queue_size=1)
            pub_receive_mbps = rospy.Publisher(rosSafeName(os.uname()[1]) + "/" + face[0] + '/receive_mbps', Float32, queue_size=1)
            queue_transmit = deque()
            queue_reveice = deque()

            for i in range(self.hz):
                queue_transmit.append(-1)
                queue_reveice.append(-1)

            self.faces_map[face[0]] = {}
            self.faces_map[face[0]]["transmit"] = {"publisher": pub_transmit, "value": queue_transmit,
                                                   "publisher_kbps": pub_transmit_kbps,
                                                   "publisher_mbps": pub_transmit_mbps}
            self.faces_map[face[0]]["receive"] = {"publisher": pub_receive, "value": queue_reveice,
                                                  "publisher_kbps": pub_receive_kbps,
                                                  "publisher_mbps": pub_receive_mbps}

    def publish(self, event):
        faces = self.read_net_file()
        non_local_transmit = 0
        non_local_receive = 0
        for face in faces:
            name = face[0]
            if name in self.faces_map:
                self.faces_map[name]["transmit"]["value"].append(int(face[1]))
                self.faces_map[name]["receive"]["value"].append(int(face[2]))
                transmit_value = (int(face[1]) - self.faces_map[name]["transmit"]["value"].popleft()) * 8
                receive_value = (int(face[2]) - self.faces_map[name]["receive"]["value"].popleft()) * 8
                self.faces_map[name]["transmit"]["publisher"].publish(Float32(transmit_value))
                self.faces_map[name]["receive"]["publisher"].publish(Float32(receive_value))
                self.faces_map[name]["transmit"]["publisher_kbps"].publish(Float32(transmit_value / 1024))
                self.faces_map[name]["receive"]["publisher_kbps"].publish(Float32(receive_value / 1024))
                self.faces_map[name]["transmit"]["publisher_mbps"].publish(Float32(transmit_value / 1024 / 1024))
                self.faces_map[name]["receive"]["publisher_mbps"].publish(Float32(receive_value / 1024 / 1024))
                if name != "lo":
                    non_local_receive = receive_value + non_local_receive
                    non_local_transmit = transmit_value + non_local_transmit
        self.non_local_transmit_pub.publish(Float32(non_local_transmit))
        self.non_local_receive_pub.publish(Float32(non_local_receive))
        self.non_local_transmit_kbps_pub.publish(Float32(non_local_transmit / 1024))
        self.non_local_receive_kbps_pub.publish(Float32(non_local_receive / 1024))
        self.non_local_transmit_mbps_pub.publish(Float32(non_local_transmit / 1024 / 1024))
        self.non_local_receive_mbps_pub.publish(Float32(non_local_receive / 1024 / 1024))
    def read_net_file(self):
        with open("/proc/net/dev") as f:
            status =  f.readlines()
        ret = []
        for s in status:
            match = self.match_face.match(s)
            if match:
                nums = match.group(2).split()
                ret.append([match.group(1).strip(), nums[8], nums[0]])
        return ret

if __name__ == '__main__':
    NetworkStatus()
