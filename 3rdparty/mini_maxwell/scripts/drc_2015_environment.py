#!/usr/bin/env python
import rospy
import sys
from mm2client import *
import setfilters
from periodic import *
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mini_maxwell.cfg import DRCEnvironmentConfig as ConfigType
from threading import Lock
from random import randint
from std_msgs.msg import Bool, Time
# constant variable for readability
LANA_to_LANB = True
LANB_to_LANA = False

class DRCEnvironment():
    def __init__(self):
        self.mm2name = rospy.get_param('~ip', '133.11.216.47')
        self.low_speed_name = "drc_low_speed"
        self.high_speed_name = "drc_high_speed"
        self.bands = Bands()
        self.lock = Lock()
        self.LOW_SPEED_BAND_NUM = 1
        self.HIGH_SPEED_BAND_NUM = 2

        self.pub_is_disabled = rospy.Publisher("~is_disabled", Bool)
        self.pub_is_blackout = rospy.Publisher("~is_blackout", Bool)
        self.pub_next_whiteout_time = rospy.Publisher("~next_whiteout_time", Time)
        # low speed -> 1
        # high speed -> 2
        self.all_filter_names = setfilters.GetAllFilterNames(self.mm2name)
        low_speed_filter = setfilters.FiltSetting(self.low_speed_name, 
                                                  self.LOW_SPEED_BAND_NUM)
                                                   
        high_speed_filter = setfilters.FiltSetting(self.high_speed_name,
                                                   self.HIGH_SPEED_BAND_NUM)
        self.filters = [low_speed_filter, high_speed_filter]
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        self.blackoutp = False
        self.next_blackout = rospy.Time.now()
        self.next_whiteout = rospy.Time.now()
        self.blackout()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publishStatus)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.updateBlackout)
        rospy.spin()
    def publishStatus(self, event):
        with self.lock:
            self.pub_is_disabled.publish(Bool(data=self.disable_network_limitation))
            self.pub_is_blackout.publish(Bool(data=self.blackoutp))
            self.pub_next_whiteout_time.publish(Time(data=self.next_whiteout))
    def updateBlackout(self, event):
        with self.lock:
            if not self.disable_network_limitation:
                now = rospy.Time.now()
                if not self.blackoutp:
                    if (self.next_blackout - now).to_sec() < 0:
                        self.blackout()
                else:
                    if (self.next_whiteout - now).to_sec() < 0:
                        self.whiteout()
    def reconfigure(self, config, level):
        with self.lock:
            self.disable_network_limitation = config["disable_network_limitation"]

            self.low_speed_link_bandwidth = config["low_speed_link_bandwidth"]

            self.high_speed_link_bandwidth = config["high_speed_link_bandwidth"]
            self.high_speed_link_duration = config["high_speed_link_duration"]
            self.high_speed_link_blackout_duration = config["high_speed_link_blackout_duration"]
            self.updateMM()
            return config
    def blackout(self):
        rospy.loginfo("blackout")
        self.bands.SetDropAmount(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB,
                                 100)
        SetMM(self.mm2name, self.bands,
              self.filters, self.filters, self.all_filter_names)
        self.blackoutp = True
        self.next_whiteout = rospy.Time.now() + rospy.Duration(randint(1, self.high_speed_link_blackout_duration))
    def whiteout(self):
        rospy.loginfo("whiteout")
        self.bands.SetDropAmount(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB,
                                 0)
        SetMM(self.mm2name, self.bands,
              self.filters, self.filters, self.all_filter_names)
        self.blackoutp = False
        self.next_blackout = rospy.Time.now() + rospy.Duration(self.high_speed_link_duration)
    def updateMM(self):
        self.bands.SetDelayAmount(self.LOW_SPEED_BAND_NUM, LANA_to_LANB, 0)
        self.bands.SetDelayAmount(self.LOW_SPEED_BAND_NUM, LANB_to_LANA, 0)
        if not self.disable_network_limitation:
            self.bands.SetRateLimit(self.LOW_SPEED_BAND_NUM, LANA_to_LANB,
                                    self.low_speed_link_bandwidth)
            self.bands.SetRateLimit(self.LOW_SPEED_BAND_NUM, LANB_to_LANA, 
                                    self.low_speed_link_bandwidth)
        else:
            self.bands.SetRateLimit(self.LOW_SPEED_BAND_NUM, LANA_to_LANB,
                                    100 * 1000 * 1000)
            self.bands.SetRateLimit(self.LOW_SPEED_BAND_NUM, LANB_to_LANA, 
                                    100 * 1000 * 1000)
        self.bands.SetDelayReorder(self.LOW_SPEED_BAND_NUM, LANB_to_LANA, False)
        self.bands.SetDelayReorder(self.LOW_SPEED_BAND_NUM, LANA_to_LANB, False)
        self.bands.SetDelayAmount(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB, 0)
        self.bands.SetDelayAmount(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA, 0)
        if not self.disable_network_limitation:
            self.bands.SetRateLimit(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB, 
                                    self.high_speed_link_bandwidth)
            self.bands.SetRateLimit(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA, 
                                    self.high_speed_link_bandwidth)
            self.bands.SetDropAmount(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA,
                                     100)
            self.bands.SetDropAmount(5, LANB_to_LANA, 0)   #default: all drop
            self.bands.SetDropAmount(5, LANA_to_LANB, 0)   #default: all drop
        else:
            self.bands.SetRateLimit(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB, 
                                    100 * 1000 * 1000)
            self.bands.SetRateLimit(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA, 
                                    100 * 1000 * 1000)
            self.bands.SetDropAmount(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA,
                                     0)
            self.bands.SetDropAmount(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB,
                                     0)
            self.bands.SetDropAmount(5, LANB_to_LANA, 0)
            self.bands.SetDropAmount(5, LANA_to_LANB, 0)
        self.bands.SetDelayReorder(self.HIGH_SPEED_BAND_NUM, LANB_to_LANA, False)
        self.bands.SetDelayReorder(self.HIGH_SPEED_BAND_NUM, LANA_to_LANB, False)
        SetMM(self.mm2name, self.bands,
              self.filters, self.filters, self.all_filter_names)
        rospy.loginfo("updated speed settings")
        
if __name__ == "__main__":
    rospy.init_node("drc_2015_environment")
    drc = DRCEnvironment()

    
