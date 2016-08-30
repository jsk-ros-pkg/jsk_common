#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, JSK Robotics Lab.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Yuki Furuta
##\brief Monitors wifi status

import re
import subprocess
import traceback

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from jsk_network_tools.msg import WifiStatus

def extract_number(s):
    try:
        extracted = re.match('^[0-9.-]*', s).group()
        try:
            return int(extracted)
        except ValueError:
            return float(extracted)
    except:
        return ''

class Iwconfig(object):
    def __init__(self, dev):
        self.dev = dev
        self.status = {}
    def fetch(self):
        try:
            output = subprocess.check_output(["iwconfig", self.dev],
                                             stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            rospy.logdebug("iwconfig command return non-zero code: %s" % str(e.returncode))
            output = e.output
        for entity in re.split('\s{2,}', output.replace('=',':').replace('"','')):
            splitted = entity.split(':')
            if len(splitted) >= 2:
                key = entity.split(':')[0].strip()
                value = ":".join(entity.split(':')[1:]).strip()
                if "Quality" in key:
                    q1, q2 = value.split('/')
                    value = str(float(q1) / float(q2))
                self.status[key] = value
    def is_enabled(self):
        if "ESSID" not in self.status:
            return False
        else:
            return True
    def is_connected(self):
        if not self.is_enabled():
            return False
        elif "off" in self.status["ESSID"]:
            return False
        else:
            return True

class NetworkStatusPublisherNode(object):
    def __init__(self):
        self.ifname = rospy.get_param("~network_interface", "wlan0")
        self.rate = rospy.get_param("~update_rate", 1.0)
        self.warn_quality = rospy.get_param("~warning_quality", 0.4)
        self.iwconfig = Iwconfig(self.ifname)
        self.status_pub = rospy.Publisher("~status", WifiStatus, queue_size=1)
        self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.poll_timer = rospy.Timer(rospy.Rate(self.rate).sleep_dur, self.poll)
    def poll(self, event=None):
        self.iwconfig.fetch()
        self.publish_status()
        self.publish_diagnostic()
    def publish_status(self):
        try:
            msg = WifiStatus()
            msg.header.stamp = rospy.Time.now()
            msg.interface = self.ifname
            msg.enabled = self.iwconfig.is_enabled()
            msg.connected = self.iwconfig.is_connected()
            if self.iwconfig.is_connected():
                msg.ssid = self.iwconfig.status["ESSID"]
                msg.frequency = extract_number(self.iwconfig.status["Frequency"])
                msg.access_point = self.iwconfig.status["Access Point"]
                msg.bitrate = extract_number(self.iwconfig.status["Bit Rate"])
                msg.tx_power = extract_number(self.iwconfig.status["Tx-Power"])
                msg.link_quality = extract_number(self.iwconfig.status["Link Quality"])
                msg.signal_level = extract_number(self.iwconfig.status["Signal level"])
            self.status_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Failed to publish status: %s" % str(e))
            rospy.logerr(traceback.format_exc())
    def publish_diagnostic(self):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = rospy.get_caller_id().lstrip('/') + ': Status'
            ds.hardware_id = self.ifname
            if not self.iwconfig.is_enabled():
                ds.level = DiagnosticStatus.STALE
                ds.message = "Device not found"
            elif not self.iwconfig.is_connected():
                ds.level = DiagnosticStatus.ERROR
                ds.message = "No connection"
            else:
                if extract_number(self.iwconfig.status["Link Quality"]) < self.warn_quality:
                    ds.level = DiagnosticStatus.WARN
                    ds.message = "Connected, but bad quality"
                else:
                    ds.level = DiagnosticStatus.OK
                    ds.message = "Connected"
                for key, val in self.iwconfig.status.items():
                    ds.values.append(KeyValue(key, val))
            da.status.append(ds)
            da.header.stamp = rospy.Time.now()
            self.diagnostic_pub.publish(da)
        except Exception as e:
            rospy.logerr('Failed to publish diagnostic: %s' % str(e))
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    rospy.init_node("network_status")
    n = NetworkStatusPublisherNode()
    rospy.spin()
