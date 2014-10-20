#!/usr/bin/env python
import roslib
roslib.load_manifest('mini_maxwell')
import rospy
import sys
from mm2client import *

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mini_maxwell.cfg import RosClientConfig as ConfigType

class MMClient():
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
        rospy.logwarn('[usage] rosrun mini_maxwell ros_client.py _ip:=[mini_maxwell ip or hostname] _round_trimp:=[round trip(ms)] _rate_limit:=[rate limit(bps)]')
        self.mm2name = rospy.get_param('~ip', '192.168.0.5')
        self.round_trip  = rospy.get_param('~round_trip', 100) #100ms
        self.rate_limit = rospy.get_param('~rate_limit', 100000) #100kbps
        self.band_number = 5
        rospy.loginfo('mm2 hostname or ip = %s', self.mm2name)

        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        rospy.spin()

    def reconfigure(self, config, level):
        self.round_trip = config["round_trip"]
        self.rate_limit = config["rate_limit"]

        # Return the new variables.
        self.updateMM()
        return config

    def updateMM(self):
        self.bnds.SetDelayAmount(self.band_number, True, self.round_trip)
        self.bnds.SetDelayAmount(self.band_number, False, self.round_trip)
        self.bnds.SetRateLimit(self.band_number, True, self.rate_limit)
        self.bnds.SetRateLimit(self.band_number, False, self.rate_limit)
        self.bnds.SetDelayReorder(self.band_number, True, False)
        self.bnds.SetDelayReorder(self.band_number, False, False)

        rospy.loginfo('round_trip = %s', self.round_trip)
        rospy.loginfo('rate_limit = %s', self.rate_limit)
        #update MM setting
        ChangeBandsOnMM(self.bnds, self.mm2name)


if __name__ == '__main__':
    rospy.init_node('mini_maxwell_client')

    try:
        mmc = MMClient()
    except rospy.ROSInterruptException: pass
