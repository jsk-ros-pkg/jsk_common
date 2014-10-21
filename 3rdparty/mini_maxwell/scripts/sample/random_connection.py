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
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mini_maxwell.cfg import RosClientConfig as ConfigType

class MMRondomConnection():
    def __init__(self):
        MM_REV = 12
        if MM_REV >= 12:
        # Read the current settings from the Mini Maxwell
            try:
                self.bnds = GetCurrentBands(mm2name)
            except:
                self.bnds = Bands()
        else: # MM_REV < 12
            self.bnds = Bands()

        # Get parameter
        self.mm2name = rospy.get_param('~ip', '192.168.0.5')
        self.round_trip  = rospy.get_param('~round_trip', 500) #500ms
        self.rate_limit = rospy.get_param('~rate_limit', 100000000) #100kbps
        self.connection_A = True
        self.connection_B = True
        self.band_number = 5

        rospy.loginfo('mm2 hostname or ip = %s', self.mm2name)

        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        rospy.Timer(rospy.Duration(1), self.changeConnection)
        rospy.spin()

    def reconfigure(self, config, level):
        self.round_trip = config["round_trip"]
        self.rate_limit = config["rate_limit"]

        # Return the new variables.
        self.updateMM()
        return config

    def changeConnection(self, event):
        present_connection_A = self.connection_A
        present_connection_B = self.connection_B

        if random.randint(0, 1):
            self.connection_A = True
            rospy.loginfo('A: Connected')
        else:
            self.connection_A = False
            rospy.loginfo('A: Not connected')

        if random.randint(0, 1):
            self.connection_B = True
            rospy.loginfo('B: Connected')
        else:
            self.connection_B = False
            rospy.loginfo('B: Not connected')
        #self.connection_A = False
        #self.connection_B = False
        self.updateMM()

    def updateMM(self):
        self.bnds.SetDelayAmount(self.band_number, True, self.round_trip)
        self.bnds.SetDelayAmount(self.band_number, False, self.round_trip)
        if self.connection_A:
            rate_limit_A = self.rate_limit
        else:
            rate_limit_A = 128 # minimum value

        if self.connection_B:
            rate_limit_B = self.rate_limit
        else:
            rate_limit_B = 128 # minimum value

        self.bnds.SetRateLimit(self.band_number, True, rate_limit_A)
        self.bnds.SetRateLimit(self.band_number, False, rate_limit_B)
        self.bnds.SetDelayReorder(self.band_number, True, False)
        self.bnds.SetDelayReorder(self.band_number, False, False)

        rospy.loginfo('round_trip = %s', self.round_trip)
        rospy.loginfo('rate_limit_A = %s', rate_limit_A)
        rospy.loginfo('rate_limit_B = %s', rate_limit_B)
        #update MM setting
        ChangeBandsOnMM(self.bnds, self.mm2name)


if __name__ == '__main__':
    rospy.init_node('mini_maxwell_rondom_connection')

    try:
        mmc = MMRondomConnection()
    except rospy.ROSInterruptException: pass
