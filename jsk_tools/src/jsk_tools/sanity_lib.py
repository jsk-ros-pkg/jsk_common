#!/usr/bin/env python
import rospy
import rosnode
import rostopic
import rosgraph
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
def indexMessage(msg):
    print Fore.LIGHTCYAN_EX + "  == %s ==" % (msg) + Fore.RESET

from sensor_msgs.msg import Image, JointState, Imu

class TopicPublishedChecker():
    is_topic_published = False
    is_topic_published_lock = Lock()
    def __init__(self, topic_name,  timeout = 5, echo = False):
        self.topic_name = topic_name
        self.timeout = timeout
        self.launched_time = rospy.Time.now()
        self.first_time_callback = True
        self.echo = echo
        print " Checking %s" % (topic_name)
        msg_class, _, _ = rostopic.get_topic_class(topic_name, blocking=True)
        self.sub = rospy.Subscriber(topic_name, msg_class, self.callback)
    def callback(self, msg):
        with self.is_topic_published_lock:
            if self.echo and self.first_time_callback:
                print Fore.MAGENTA + "--- Echo "+ self.topic_name, Fore.RESET
                self.first_time_callback = False
                field_filter_fn = rostopic.create_field_filter(False, True)
                callback_echo = rostopic.CallbackEcho(self.topic_name, None, plot=False,
                                                      filter_fn=None,
                                                      echo_clear=False, echo_all_topics=False,
                                                      offset_time=False, count=None,
                                                      field_filter_fn=field_filter_fn)
                print Fore.CYAN
                callback_echo.callback(msg, {"topic":self.topic_name,
                                             "type_infomation":None})
                print Fore.RESET
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
                          other_topics = [],
                          echo = False):
    checkers = []
    checkers.append(TopicPublishedChecker(topic_name, timeout, echo))
    if other_topics:
        for (tpc_name, cls) in other_topics:
            checkers.append(TopicPublishedChecker(tpc_name, timeout, echo))
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

def isMasterHostAlive(host, ok_message="", error_message=""):
    response = os.system("ping -W 10 -c 1 " + host + " > /dev/null")
    if response != 0:
        errorMessage( error_message if error_message else "%s is down" % host )
        return False
    else:
        okMessage( ok_message if ok_message else "%s is up" % host)
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

def checkLocalRemoteROSParamDiff(param_name, host, ok_message="", error_message=""):
    if not rospy.has_param(param_name):
        errorMessage("Local doesn't have " + param_name)
        return False

    target_local_param_md5 = subprocess.check_output("bash -c 'source ~/ros/indigo/devel/setup.bash ;rosrun jsk_tools calc_md5.py " + param_name + " '", shell=True)
    target_remote_param_md5 = subprocess.check_output(["ssh", host,  "source ~/ros/indigo/devel/setup.bash; rosrun jsk_tools calc_md5.py "+param_name])

    if target_local_param_md5 == target_remote_param_md5:
        okMessage("Local and Remote ("+host+") 's "+param_name+" is same and is " + str(rospy.get_param(param_name)))
    else:
        errorMessage("Local and Remote ("+host+") 's "+param_name+" is DIFFERENT !! (Local value is " + str(rospy.get_param(param_name)) + ")")

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

def checkUSBExist(vendor_id, product_id, expect_usb_nums = 1, host="", success_msg = "", error_msg = ""):
    """check USB Exists
    
    vendor_string -- vendor string (e.g. if 8087:0024, 8087)
    product_string -- product string (e.g. if 8087:0024, 0024)
    expect_usb_nums -- number of usbs (default is 1)
    """
    vendor_product = str(vendor_id) + ":" + str(product_id)
    print Fore.LIGHTCYAN_EX + "Check USB Connect " + vendor_product + " x"+str(expect_usb_nums) + (" in "+str(host) if host else "")
    output_lines = ""
    if host:
        output_lines = subprocess.check_output("ssh "+ host +" lsusb", shell=True).split("\n")
    else:
        output_lines = subprocess.check_output("lsusb", shell=True).split("\n")
    usb_counter = 0
    for output in output_lines:
        if vendor_product in output:
            usb_counter += 1
    if usb_counter == expect_usb_nums:
        okMessage(vendor_product + " ("+str(usb_counter)+"/"+str(expect_usb_nums)+"): " + success_msg if success_msg else vendor_product + " Found enough ( "+str(usb_counter)+"/"+str(expect_usb_nums)+" ) Detected")
        return True
    elif usb_counter == 0:
        errorMessage(vendor_product + " (0/" + str(expect_usb_nums) + ")  " + error_msg if error_msg else vendor_product + " NOT Found !! ( 0 / " + str(expect_usb_nums) + " ) Detected")
        return False
    else:
        errorMessage(vendor_product + " ("+str(usb_counter)+"/"+str(expect_usb_nums)+") " + error_msg if error_msg else vendor_product + " DOESN'T MATCH !! ( "+str(usb_counter)+"/"+str(expect_usb_nums)+" ) Detected")
        return False
            
def checkROSMasterCLOSE_WAIT(host, username=""):
    try:
        if username != "":
            host = username + "@" + host
        close_wait_num = int(subprocess.check_output(["ssh", host, "sudo", "bash", "-c", '"ps aux | grep rosmaster | grep CLOSE_WAIT | wc -l"']).split("\n")[0])
        if close_wait_num < 150:
            okMessage("roscore looks find (%d CLOSE_WAIT)" % close_wait_num)
            return True
        elif close_wait_num < 300:
            warnMessage("roscore looks bad (%d CLOSE_WAIT)" % close_wait_num)
            return True
        else:
            errorMessage("roscore looks not working (%d CLOSE_WAIT)" % close_wait_num)
            return False
    except:
        errorMessage("failed to check CLOSE_WAIT")
        return False

class SilverHammerSubscribeChecker():
    is_topic_published = False
    is_topic_published_lock = Lock()
    def __init__(self, topic_name, timeout, expected_hz, expected_error_threshold, until_counter = 3):
        self.expected_hz = expected_hz
        self.expected_error_threshold = expected_error_threshold
        self.prev_time = None
        self.until_counter = until_counter
        self.counter = 0
        self.success = True

        self.topic_name = topic_name
        self.timeout = timeout
        self.launched_time = rospy.Time.now()
        self.first_time_callback = True
        print Fore.LIGHTCYAN_EX," Checking %s %s times" % (topic_name, str(until_counter)), Fore.RESET
        msg_class, _, _ = rostopic.get_topic_class(topic_name, blocking=True)
        self.sub = rospy.Subscriber(topic_name, msg_class, self.callback)


    def callback(self, msg):
        if self.prev_time:
            diff = (rospy.Time.now() - self.prev_time).to_sec()
            rate = 0.0
            if diff:
                rate = 1.0/diff
            if diff and (self.expected_hz - self.expected_error_threshold > rate or rate > self.expected_hz + self.expected_error_threshold):
                errorMessage("Estimated Hz is OUT OF EXPECTED!! min:" + str(self.expected_hz - self.expected_error_threshold) + " max: " + str(self.expected_hz + self.expected_error_threshold) + " real: " + str(rate) )
                self.success = False
            elif diff:
                print Fore.LIGHTMAGENTA_EX, "    ["+ str(self.counter+1)+"] Recieved Topic (" + str(self.topic_name) + ") Estimated hz : " + str(rate) + " Expected hz : " + str(self.expected_hz) +  "+-" + str(self.expected_error_threshold)
            else:
                print "duration was 0"
        else:
            print Fore.LIGHTMAGENTA_EX, "    ["+ str(self.counter+1)+"] Recieved Topic (" + str(self.topic_name) + ") First Time Recieved"
        self.prev_time = rospy.Time.now()

        self.counter += 1
        with self.is_topic_published_lock:
            if self.counter >= self.until_counter:
                self.is_topic_published = True

    def check(self):
        try:
            while not rospy.is_shutdown():
                with self.is_topic_published_lock:
                    if self.is_topic_published:
                        return self.success
                if (rospy.Time.now() - self.launched_time).to_sec() > self.timeout:
                    errorMessage("Timeout has occured in ("+self.topic_name+"). Timeout is set as "+ str(self.timeout) + " sec")
                    return False
        finally:
            self.sub.unregister()


## USAGE ####
# checkSilverHammerSubscribe("/ocs_from_fc_low_speed/output", 1 , 0.5)
def checkSilverHammerSubscribe(topic_name, expected_hz, expected_error_threshold,
                               ok_message = "",
                               error_message = "",
                               timeout = 20,
                               other_topics = [],
                               count_time = 3):
    checkers = []
    checkers.append(SilverHammerSubscribeChecker(topic_name, timeout, expected_hz, expected_error_threshold, until_counter = count_time))
    if other_topics:
        for (tpc_name, cls) in other_topics:
            checkers.append(SilverHammerSubscribeChecker(tpc_name, timeout, expected_hz, expected_error_threshold, until_counter = count_time))
    all_success = True
    for checker in checkers:
        if not checker.check():
            all_success = False
    if not all_success:
        if error_message:
            errorMessage(error_message)
        else:
            erorrMessage(" Topic %s 's Hz is BAD. Check SilverHammerNode. " % (topic_name))
        return False
    if ok_message:
        if ok_message:
            okMessage(ok_message)
        else:
            okMessage(" Topic %s 's Hz is well. " % (topic_name))
    return True

def checkBlackListDaemon(daemon_names, kill=False):
#    indexMessage("Check BlackLists Daemon [%s]" % (str(daemon_names)))
    for daemon_name in daemon_names:
        daemon_related_nums = int(subprocess.check_output("ps aux | grep " + daemon_name + " | wc |tr -s ' '| cut -f2 -d ' '", shell=True).strip("\n")) - 2
        if daemon_related_nums > 0:
            errorMessage("There is a BAD Daemon " + daemon_name + ". (" + str(daemon_related_nums) + " process)")
            if kill:
                print "pKilling " + daemon_name + " ... "
                subprocess.check_output("pkill " + daemon_name, shell=True)
                import time
                time.sleep(2)
                
                daemon_related_nums = int(subprocess.check_output("ps aux | grep " + daemon_name + " | wc |tr -s ' '| cut -f2 -d ' '", shell=True).strip("\n")) - 2
                if daemon_related_nums > 0:
                    errorMessage("a BAD Daemon " + daemon_name + " STILL exist. (" + str(daemon_related_nums) + " process). Please kill by yourself")
                else:
                    okMessage("Now all BAD Daemon" + daemon_name + " is removed!")

        else:
            okMessage("No BAD Daemon " + daemon_name + " found.")
