#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

keep_flag = True
results = {}

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def callback(data):
    global keep_flag, results
    status = data.status
    for s in status:
        if s.name.startswith("/Power System/Smart Battery"):
            for key_value in s.values:
                if(key_value.key.startswith("Full Charge Capacity (mAh)")):
                    results[s.name] = {
                        "value":key_value.value,
                        "hardware_id":s.hardware_id
                        }
    keep_flag = False

def output():
    global results
    print " |         Battery Name          | value|hardware_id|" + bcolors.ENDC
    for key,v in results.items():
        value = int(v["value"])
        if value > 5500:
            print bcolors.OKGREEN,
        elif value > 4000:
            print bcolors.WARNING,
        else:
            print bcolors.FAIL,
        print "|" + key + "| " + str(value) + " |   " + str(v["hardware_id"]) + "   |" + bcolors.ENDC


if __name__ == '__main__':
    rospy.init_node('battery_summary')
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, callback)

    while not rospy.is_shutdown() and keep_flag:
        rospy.sleep(1)

    output()
