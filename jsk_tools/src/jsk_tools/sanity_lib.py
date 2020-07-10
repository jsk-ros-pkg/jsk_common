#!/usr/bin/env python
import rospy
import rosnode
import rostopic
import rosgraph
import rospkg
import os
import subprocess
from threading import Lock
import sys
import math
try:
    import colorama
except:
    print("""Please install colorama by
pip install colorama""")
    sys.exit(1)
from colorama import Fore, Style

def okMessage(msg):
    print(Fore.GREEN + "[OK]    %s" % (msg) + Fore.RESET)
def errorMessage(msg):
    print(Fore.RED + "[ERROR] %s" % (msg) + Fore.RESET)
def warnMessage(msg):
    print(Fore.YELLOW + "[WARN]  %s" % (msg) + Fore.RESET)
def indexMessage(msg):
    print("")
    print(Fore.LIGHTCYAN_EX + "  == %s ==" % (msg) + Fore.RESET)

import genpy.message
from sensor_msgs.msg import Image, JointState, Imu

def colored(string, color):
    colors = {
        'clear': '\033[0m',
        'black': '\033[30m',
        'red': '\033[31m',
        'green': '\033[32m',
        'yellow': '\033[33m',
        'blue': '\033[34m',
        'purple': '\033[35m',
        'cyan': '\033[36m',
        'white': '\033[37m'
        }
    if color in colors:
        return colors[color] + string + colors['clear']
    else:
        return string


class TopicPublishedChecker(object):

    """Utility class to check if topic is published"""

    def __init__(self, topic_name, timeout=5, echo=False,
                 data_class=None, echo_noarr=False):
        self.msg = None
        self.topic_name = topic_name
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        self.echo = echo
        self.echo_noarr = echo_noarr
        print(' Checking %s for %d seconds' % (topic_name, timeout))
        msg_class, _, _ = rostopic.get_topic_class(topic_name, blocking=True)
        if (data_class is not None) and (msg_class is not data_class):
            raise rospy.ROSException('Topic msg type is different.')
        self.sub = rospy.Subscriber(topic_name, msg_class, self.callback)

    def callback(self, msg):
        if self.echo and self.msg is None:  # this is first time
            print(colored('--- Echo %s' % self.topic_name, 'purple'))
            field_filter = rostopic.create_field_filter(echo_nostr=False, echo_noarr=self.echo_noarr)
            print(colored(genpy.message.strify_message(msg, field_filter=field_filter), 'cyan'))
        if rospy.Time.now() < self.deadline:
            self.msg = msg

    def check(self):
        while not rospy.is_shutdown():
            if self.msg is not None:
                return True
            elif rospy.Time.now() > self.deadline:
                return False
            else:
                rospy.sleep(0.1)


def checkTopicIsPublished(topic_name, class_name = None,
                          ok_message = "",
                          error_message = "",
                          timeout = 1,
                          other_topics = [],
                          echo = False,
                          echo_noarr = False):
    """
    @type class_name: type
    @property class_name:
        ROS message data class.
        if not None, it checks if msg type is same as published one.
    """
    checkers = []
    checkers.append(TopicPublishedChecker(topic_name, timeout, echo,
                                          data_class=class_name,
                                          echo_noarr=echo_noarr))
    if other_topics:
        for (tpc_name, cls) in other_topics:
            checkers.append(TopicPublishedChecker(tpc_name, timeout, echo,
                                                  data_class=class_name))
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
                print(Fore.GREEN+"    "+sub_success+ Fore.RESET)
        else:
            errorMessage("Node " + target_node_name + " exists unexpecetedly. This should be killed with rosnode kill")
            if sub_fail:
                print(Fore.RED+"    "+sub_fail+ Fore.RESET)
    else:
        if needed:
            errorMessage("Node " + target_node_name + " doesn't exists. This node is NEEDED")
            if sub_fail:
                print(Fore.RED+"    "+sub_fail+ Fore.RESET)
        else:
            okMessage("Node " + target_node_name + " doesn't exists")
            if sub_success:
                print(Fore.GREEN+"    "+sub_success+ Fore.RESET)

def checkUSBExist(vendor_id, product_id, expect_usb_nums = 1, host="", success_msg = "", error_msg = ""):
    """check USB Exists
    
    vendor_string -- vendor string (e.g. if 8087:0024, 8087)
    product_string -- product string (e.g. if 8087:0024, 0024)
    expect_usb_nums -- number of usbs (default is 1)
    """
    vendor_product = str(vendor_id) + ":" + str(product_id)
    print(Fore.LIGHTCYAN_EX + "Check USB Connect " + vendor_product + " x"+str(expect_usb_nums) + (" in "+str(host) if host else ""))
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


def getROSMasterCLOSE_WAIT(host, username=""):
    if username != "":
        host = username + "@" + host
    return int(subprocess.check_output(["ssh", host, "sudo", "bash", "-c", '"lsof -l | grep rosmaster | grep CLOSE_WAIT | wc -l"']).split("\n")[0])

def checkROSMasterCLOSE_WAIT(host, username=""):
    try:
        close_wait_num = getROSMasterCLOSE_WAIT(host, username)
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
        print(Fore.LIGHTCYAN_EX," Checking %s %s times" % (topic_name, str(until_counter)), Fore.RESET)
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
                print(Fore.LIGHTMAGENTA_EX, "    ["+ str(self.counter+1)+"] Recieved Topic (" + str(self.topic_name) + ") Estimated hz : " + str(rate) + " Expected hz : " + str(self.expected_hz) +  "+-" + str(self.expected_error_threshold))
            else:
                print("duration was 0")
        else:
            print(Fore.LIGHTMAGENTA_EX, "    ["+ str(self.counter+1)+"] Recieved Topic (" + str(self.topic_name) + ") First Time Recieved")
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
            errorMessage(" Topic %s 's Hz is BAD. Check SilverHammerNode. " % (topic_name))
        return False
    if ok_message:
        okMessage(ok_message)
    else:
        okMessage(" Topic %s 's Hz is well. " % (topic_name))
    return True

def checkBlackListDaemon(daemon_names, kill=False):
#    indexMessage("Check BlackLists Daemon [%s]" % (str(daemon_names)))
    for daemon_name in daemon_names:
        daemon_related_nums = int(subprocess.check_output("ps aux | grep " + daemon_name + " | grep -v grep | wc | tr -s ' ' | cut -f2 -d ' '", shell=True).strip("\n")) - 2
        if daemon_related_nums > 0:
            errorMessage("There is a BAD Daemon " + daemon_name + ". (" + str(daemon_related_nums) + " process)")
            if kill:
                print("pKilling " + daemon_name + " ... ")
                subprocess.check_output("pkill " + daemon_name, shell=True)
                import time
                time.sleep(2)
                
                daemon_related_nums = int(subprocess.check_output("ps aux | grep " + daemon_name + " | grep -v grep | wc |tr -s ' '| cut -f2 -d ' '", shell=True).strip("\n")) - 2
                if daemon_related_nums > 0:
                    errorMessage("a BAD Daemon " + daemon_name + " STILL exist. (" + str(daemon_related_nums) + " process). Please kill by yourself")
                else:
                    okMessage("Now all BAD Daemon" + daemon_name + " is removed!")

        else:
            okMessage("No BAD Daemon " + daemon_name + " found.")

class ROSMasterChecker():
    def __init__(self):
        self.rosmater_pid = None
        self.cur_tracking_pid = None

    def getProcessNameFromPID(self, pid):
        return subprocess.check_output("ps -p  %s -o comm= " % (str(pid)), shell=True).strip("\n")

    def getParentPID(self, pid):
        return subprocess.check_output("ps -p  " + str(pid) + " -oppid= ", shell=True).strip("\n")

    def getMasterPID(self):
        self.rosmaster_pid = subprocess.check_output("ps aux | grep rosmaster | tr -s ' ' | grep -v grep |cut -f 2 -d ' ' |  sed -n '1p'", shell=True).strip("\n")

    def checkIsFromRoscore(self):
        self.getMasterPID()
        self.cur_tracking_pid = self.rosmaster_pid
        cur_tracking_pname = self.getProcessNameFromPID(self.cur_tracking_pid)
        while not cur_tracking_pname in [ "bash", "gnome-terminal"]:
            cur_tracking_pname = self.getProcessNameFromPID(self.cur_tracking_pid)
            if cur_tracking_pname == "roslaunch":
                errorMessage("This rosmaster is generated by ROSLAUNCH !!")
                return False
            self.cur_tracking_pid = self.getParentPID(self.cur_tracking_pid)
        else:
            okMessage("This rosmaster is generated by roscore")
            return True

def checkROSCoreROSMaster():
    rmc = ROSMasterChecker()
    rmc.checkIsFromRoscore()

def colored(string, color):
    colors = {
        'clear': '\033[0m',
        'black': '\033[30m',
        'red': '\033[31m',
        'green': '\033[32m',
        'yellow': '\033[33m',
        'blue': '\033[34m',
        'purple': '\033[35m',
        'cyan': '\033[36m',
        'white': '\033[37m'
        }
    if color in colors:
        return colors[color] + string + colors['clear']
    else:
        return string

from operator import add

def isROSWS():
    if "ROS_WORKSPACE" in os.environ:
        ROS_WORKSPACE = os.environ["ROS_WORKSPACE"]
        return (ROS_WORKSPACE and 
                os.path.exists(os.path.join(ROS_WORKSPACE, ".rosinstall")))
    else:
        return False

def splitPathEnv(env_name):
    return env_name.split(":")

def estimateROSPackagePath(path):
    if path.endswith("devel"):
        return [path, os.path.join(path, "..", "src")]
    else:
        return [os.path.join(path, "share"), os.path.join(path, "stacks")]

def checkROSPackagePath():
    ROS_PACKAGE_PATH = splitPathEnv(os.environ["ROS_PACKAGE_PATH"])
    CMAKE_PREFIX_PATH = splitPathEnv(os.environ["CMAKE_PREFIX_PATH"])
    estimated_ros_package_path = [os.path.abspath(p) 
                                  for p in 
                                  reduce(add, [estimateROSPackagePath(c) 
                                               for c in CMAKE_PREFIX_PATH])]
    dubious_paths = []
    for p in ROS_PACKAGE_PATH:
        normalized_path = os.path.abspath(p)
        if not normalized_path in estimated_ros_package_path:
            if isROSWS():
                ROS_WORKSPACE = os.path.abspath(os.environ["ROS_WORKSPACE"])
                if not p.startswith(ROS_WORKSPACE):
                    dubious_paths.append(p)
    print(colored("[ROS_PACKAGE_PATH]", "green"))
    if dubious_paths:
        print("  ", colored("these path might be malformed: ", "red"))
        for p in dubious_paths:
            print("    ", colored(p, "red"))
    else:
        print("  ", colored("ROS_PACKAGE_PATH seems to be OK", "cyan"))


def checkGitRepoDiff(git_path):
    os.chdir(git_path)
    output = subprocess.check_output(["git", "status", "--short"])
    modified_files = []
    if output:
        for l in output.split("\n"):
            if l.startswith(" M"):
                modified_files.append(l)
    if modified_files:
        errorMessage("  %d files are modified" % (len(modified_files)))
        for f in modified_files:
            print("    ", colored(f, "red"))
    else:
        okMessage("  No modified files")

def checkGitBranch(git_path):
    os.chdir(git_path)
    current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref",
                                   "HEAD"]).strip()
    try:
        with open(os.devnull, "w") as devnull:
            current_tracking_branch = subprocess.check_output(["git", "rev-parse",
                                                    "--abbrev-ref",
                                                    "--symbolic-full-name", "@{u}"],
                                                    stderr=devnull).strip()
    except subprocess.CalledProcessError:
        current_tracking_branch = None
    remote_origin_head = subprocess.check_output(["git", "rev-parse", "--abbrev-ref",
                                       "origin/HEAD"]).strip()
    if not current_tracking_branch:
        errorMessage("no tracking branch")
    elif current_tracking_branch != remote_origin_head:
        errorMessage("the branch(%s) may not sync with %s"
                              % (current_branch,
                              remote_origin_head))
    else:
        okMessage("  No Branch Problem")

def checkGitRepo(git_path):
    print(colored("[checking %s]" % git_path, "green"))
    checkGitRepoDiff(git_path)
    checkGitBranch(git_path)

def checkGitRepoWithRosPack(rospackage_name):
    checkGitRepo(rospkg.RosPack().get_path(rospackage_name))

def checkWorkspace():
    workspace = None
    if isROSWS():
        workspaces = [os.environ["ROS_WORKSPACE"]]
    else:
        workspaces = [os.path.abspath(os.path.join(p, "..", "src"))
                      for p in splitPathEnv(os.environ["CMAKE_PREFIX_PATH"])]
        workspaces.reverse()
    git_repos = []
    for workspace in workspaces:
        for root, dirs, files in os.walk(workspace):
            if ".git" in dirs:                  
                if not [repo for repo in git_repos if root.startswith(repo)]: #ignore subdirs
                    git_repos.append(root)
                    checkGitRepo(root)

def checkNetworkSpeed(expected_speed, ok_message="", error_message=""):
    interfaces = subprocess.check_output( "ifconfig  | tr -s ' ' | cut -f 1 -d ' ' | xargs echo", shell=True).split(" ")
    eths=[]
    for interface in interfaces:
        if "eth" in interface:
            try:
                speed_string = subprocess.check_output( "ethtool "+str(interface)+" 2>/dev/null | grep 'Speed: ' | cut -f 2 -d ' ' | grep -oE '[0-9]{0,}'", shell=True)
                if speed_string:
                    speed = int(speed_string)
                else:
                    raise subprocess.CalledProcessError("", "", "")
                if speed == expected_speed:
                    okMessage(ok_message if ok_message else "Network Speed for " + interface + " is Expected Speed " + str(speed) + " MB/s" )
                    eths.append(interface)
                else:
                    errorMessage(error_message if error_message else "Network Speed for " + interface + " is NOT EXPECTED!! Expect : " + str(expected_speed) + " MB/s Real : " + str(speed) + " MB/s" )
                    return False
            except subprocess.CalledProcessError:
                errorMessage("Is eth connected?")
                return False
    else:
        if len(eths) == 0:
            warnMessage("The eth wan not found through ifconfig")
        return True
