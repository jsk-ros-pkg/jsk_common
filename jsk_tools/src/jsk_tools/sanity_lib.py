#!/usr/bin/env python
import rospy
import rosnode
import os
import subprocess
from threading import Lock
import sys
import math
try:
    import colorama
except:
    print """Please install colorama by
pip install colorama"""
    sys.exit(1)
from colorama import Fore, Style

def okMessage(msg):
    print Fore.GREEN + "[OK]    %s" % (msg) + Fore.RESET
def errorMessage(msg):
    print Fore.RED + "[ERROR] %s" % (msg) + Fore.RESET
def warnMessage(msg):
    print Fore.YELLOW + "[WARN]  %s" % (msg) + Fore.RESET
from sensor_msgs.msg import Image, JointState, Imu

class TopicPublishedChecker():
    is_topic_published = False
    is_topic_published_lock = Lock()
    def __init__(self, topic_name, topic_class, timeout = 5):
        self.topic_name = topic_name
        self.timeout = timeout
        self.launched_time = rospy.Time.now()
        print " Checking %s" % (topic_name)
        self.sub = rospy.Subscriber(topic_name, topic_class, self.callback)
    def callback(self, msg):
        with self.is_topic_published_lock:
            self.is_topic_published = True
    def check(self):
        try:
            while not rospy.is_shutdown():
                with self.is_topic_published_lock:
                    if self.is_topic_published:
                        return self.is_topic_published
                if (rospy.Time.now() - self.launched_time).to_sec() > self.timeout:
                    return False
        finally:
            self.sub.unregister()

def checkTopicIsPublished(topic_name, class_name,
                          ok_message = "",
                          error_message = "",
                          timeout = 1,
                          other_topics = []):
    checkers = []
    checkers.append(TopicPublishedChecker(topic_name, class_name, timeout))
    if other_topics:
        for (tpc_name, cls) in other_topics:
            checkers.append(TopicPublishedChecker(tpc_name, cls, timeout))
    all_success = True
    for checker in checkers:
        if not checker.check():
            errorMessage(" %s is not published" % checker.topic_name)
            all_success = False
    if not all_success:
        if error_message:
            errorMessage(error_message)
        return False
    if ok_message:
        okMessage(ok_message)
    return True

def isMasterHostAlive(host):
    response = os.system("ping -W 10 -c 1 " + host + " > /dev/null")
    if response != 0:
        errorMessage("%s is down" % host)
        return False
    else:
        okMessage("%s is up" % host)
        return True

is_imu_ok = None
is_imu_ok_lock = Lock()
    
def checkIMUCallback(msg):
    global is_imu_ok
    with is_imu_ok_lock:
        if (not math.isnan(msg.orientation.x) and
            not math.isnan(msg.orientation.y) and
            not math.isnan(msg.orientation.z) and
            not math.isnan(msg.orientation.w)):
            is_imu_ok = True
        else:
            is_imu_ok = False
            
def checkIMU(timeout=5):
    global is_imu_ok
    s = rospy.Subscriber("/imu", Imu, checkIMUCallback)
    start_time = rospy.Time.now()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        diff = (now - start_time).to_sec()
        if diff > timeout:
            break
        with is_imu_ok_lock:
            if is_imu_ok != None:
                break
        rate.sleep()
    with is_imu_ok_lock:
        try:
            if is_imu_ok:
                okMessage("IMU looks working")
                return True
            else:
                errorMessage("IMU contains NaN. Please reset kf plugin")
                return False
        finally:
            s.unregister()

def checkROSParams(param_name, expected, needed=False):
    if not rospy.has_param(param_name):
        if needed:
            errorMessage("Parameter " + param_name + " doesn't exist" )
        else:
            warnMessage("Parameter " + param_name + " doesn't exist. Expected value was " + str(expected))
        return

    target_param = rospy.get_param(param_name)
    if target_param == expected:
        okMessage("Parameter " + param_name + " is " + str(expected))
    else:
        errorMessage("Parameter " + param_name + " is " + str(target_param) + ". Doesn't match with exepcted value : " + str(expected))


def checkNodeState(target_node_name, needed, sub_success="", sub_fail=""):
    nodes = rosnode.get_node_names()
    if target_node_name in nodes:
        if needed:
            okMessage("Node " + target_node_name + " exists")
            if sub_success:
                print Fore.GREEN+"    "+sub_success+ Fore.RESET
        else:
            errorMessage("Node " + target_node_name + " exists unexpecetedly. This should be killed with rosnode kill")
            if sub_fail:
                print Fore.RED+"    "+sub_fail+ Fore.RESET            
    else:
        if needed:
            errorMessage("Node " + target_node_name + " doesn't exists. This node is NEEDED")
            if sub_fail:
                print Fore.RED+"    "+sub_fail+ Fore.RESET
        else:
            okMessage("Node " + target_node_name + " doesn't exists")
            if sub_success:
                print Fore.GREEN+"    "+sub_success+ Fore.RESET


def checkUSBExist(vendor_id, product_id, success_msg = "", error_msg = ""):
    """check USB Exists
    
    vendor_string -- vendor string (e.g. if 8087:0024, 8087)
    product_string -- product string (e.g. if 8087:0024, 0024)
    """
    output_lines = subprocess.check_output("lsusb", shell=True).split("\n")
    vendor_product = str(vendor_id) + ":" + str(product_id)
    for output in output_lines:
        if vendor_product in output:
            okMessage(vendor_product + " " + success_msg if success_msg else vendor_product + " Found")
            return True
    else:
        errorMessage(vendor_product + " " + error_msg if error_msg else vendor_product + " NOT Found !!")
        return False
            
