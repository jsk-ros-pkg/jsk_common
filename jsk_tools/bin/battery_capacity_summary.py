#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
try:
    from colorama import Fore, Style, init
except:
  print "Please install colorama by pip install colorama"
  sys.exit(1)


keep_flag = True
results = {}

def callback(data):
    global keep_flag, results
    status = data.status
    for s in status:
        if s.name.startswith("/Power System/Smart Battery"):
            if s.name not in results:
                results[s.name] = { "HardwareID": s.hardware_id }
            for kv in s.values:
                if(kv.key.startswith("Full Charge Capacity (mAh)")):
                    results[s.name]["FullCapacity"] = int(kv.value)
                elif kv.key.startswith("Remaining Capacity"):
                    results[s.name]["RemainingCapacity"] = int(kv.value)
                elif kv.key.startswith("Batery Status"):
                    results[s.name]["Status"] = int(kv.value)
                elif kv.key.startswith("Cycle Count"):
                    results[s.name]["CycleCount"] = int(kv.value)
                elif kv.key.startswith("Manufacture Date"):
                    results[s.name]["ManufactureDate"] = kv.value
    keep_flag = False

def getColor(result):
    try:
        cap = result["FullCapacity"]
        if cap > 5500:
            return Fore.GREEN
        elif cap > 4000:
            return Fore.YELLOW
        else:
            return Fore.RED
    except:
        return ""

def output():
    global results
    sorted_names = sorted(results)
    fmt = "{:>31}" + ("|{:>15}" * len(results[sorted_names[0]]))
    sorted_keys = sorted(results[sorted_names[0]])
    print fmt.format("Battery Name", *sorted_keys)
    for name in sorted_names:
        color = getColor(results[name])
        v = [results[name][k] if k in results[name] else "N/A" for k in sorted_keys]
        print color + fmt.format(name, *v) + Fore.RESET

if __name__ == '__main__':
    init()
    rospy.init_node('battery_summary')
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, callback)

    while not rospy.is_shutdown() and keep_flag:
        print "aggregating battery info..."
        rospy.sleep(1)

    output()
