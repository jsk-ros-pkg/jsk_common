#!/usr/bin/env python
import rospy
import os
from threading import Lock


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


is_topic_published = False
is_topic_published_lock = Lock()

def checkTopicIsPublishedCallback(msg):
    global is_topic_published
    with is_topic_published_lock:
        is_topic_published = True

def checkTopicIsPublished(topic_name, class_name,
                          ok_message = "",
                          error_message = "",
                          timeout = 1):
    global is_topic_published
    is_topic_published = False
    print  Fore.RESET + "  Checking %s" % (topic_name) + Fore.RESET
    s = rospy.Subscriber(topic_name, class_name,
                         checkTopicIsPublishedCallback)
    start_time = rospy.Time.now()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        diff = (now - start_time).to_sec()
        if diff > timeout:
            break
        with is_topic_published_lock:
            if is_topic_published:
                break
        rate.sleep()
    with is_topic_published_lock:
        try:
            if is_topic_published:
                okMessage("%s is published" % (topic_name))
                if ok_message:
                    okMessage(ok_message)
            else:
                errorMessage("%s is not published" % (topic_name))
                if error_message:
                    errorMessage(error_message)
            return is_topic_published
        finally:
            is_topic_published = False
            s.unregister()

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

