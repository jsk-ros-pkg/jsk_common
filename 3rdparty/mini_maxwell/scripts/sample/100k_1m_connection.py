#!/usr/bin/env python
import roslib
roslib.load_manifest('mini_maxwell')
import rospy
import sys
import random
import os

import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('mini_maxwell') + "/scripts")

from mm2client import *
from ros_client import MMClient
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mini_maxwell.cfg import RosClientConfig as ConfigType

class MMSwitchingConnection(MMClient):
    def __init__(self):
        self.use_A = True

        self.rate_limit_A = 1000 * 1000 #1Mbps
        self.rate_limit_B = 100  * 1000 #100kbps
        self.round_trip_A = 0
        self.round_trip_B = 0

        rospy.set_param('~rate_limit', self.rate_limit_A)
        rospy.set_param('~round_trip', self.round_trip_A)

        rospy.Timer(rospy.Duration(10), self.changeConnection)

        MMClient.__init__(self)

    def changeConnection(self, event):
        self.use_A = not self.use_A
        if self.use_A:
            self.rate_limit = self.rate_limit_A
            self.round_trip = self.round_trip_A
        else:
            self.rate_limit = self.rate_limit_B
            self.round_trip = self.round_trip_B
        self.updateMM()


if __name__ == '__main__':
    rospy.init_node('mini_maxwell_switching_connection')

    try:
        mmc = MMSwitchingConnection()
    except rospy.ROSInterruptException: pass
