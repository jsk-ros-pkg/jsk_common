#!/usr/bin/env python
import os
import rospy
import diagnostic_updater
import diagnostic_msgs.msg
from urlparse import urlparse
from jsk_tools.sanity_lib import getROSMasterCLOSE_WAIT

close_wait_num = 0

def timerCallback(event):
    global close_wait_num
    close_wait_num = getROSMasterCLOSE_WAIT(hostname, user)

def diagnosticCallback(event):
    updater.update()
    
def diagnostics(stat):
    if close_wait_num < 150:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                     "CLOSE_WAIT is %d" % (close_wait_num))
    elif close_wait_num < 300:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                     "CLOSE_WAIT is %d" % (close_wait_num))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                     "CLOSE_WAIT is %d" % (close_wait_num))
    stat.add("CLOSE_WAIT", close_wait_num)
    return stat
if __name__ == "__main__":
    rospy.init_node("monitor_roscore")
    check_rate = rospy.get_param("~rate", 1)
    hostname = urlparse(os.environ["ROS_MASTER_URI"]).hostname
    user = rospy.get_param("~user", "")
    updater = diagnostic_updater.Updater()
    updater.setHardwareID("none")
    updater.add("roscore", diagnostics)
    rospy.Timer(rospy.Duration(1.0 / check_rate), timerCallback)
    rospy.Timer(rospy.Duration(1), diagnosticCallback)
    rospy.spin()
    
