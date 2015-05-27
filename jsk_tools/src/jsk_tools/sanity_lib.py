#!/usr/bin/env python
import rospy
import os
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
    for checker in checkers:
        if not checker.check():
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

