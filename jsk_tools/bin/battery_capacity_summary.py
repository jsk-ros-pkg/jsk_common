#!/usr/bin/env python

from collections import OrderedDict
import sys
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
try:
    from colorama import Fore, Style, init
except:
  print("Please install colorama by pip install colorama")
  sys.exit(1)


keep_flag = True
results = {}
data_column = OrderedDict()
data_column["Serial"] = "Serial Number"
data_column["ManufactureDate"] = "Manufacture Date"
data_column["FullCapacity(mAh)"] = "Full Charge Capacity (mAh)"
data_column["RemainingCapacity(mAh)"] = "Remaining Capacity (mAh)"
data_column["Voltage(mV)"] = "Voltage (mV)"
data_column["CycleCount"] = "Cycle Count"
data_column["Status"] = "Battery Status"

def callback(data):
    global keep_flag, results
    for s in data.status:
        if s.name.startswith("/Power System/Smart Battery"):
            if s.name not in results:
                results[s.name] = {}
            for col, label in data_column.items():
                for kv in s.values:
                    if kv.key.startswith(label):
                        try:
                            results[s.name][col] = int(kv.value)
                        except:
                            results[s.name][col] = kv.value
                        continue
    keep_flag = False

def getColor(result):
    try:
        cap = result["FullCapacity(mAh)"]
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
    sorted_keys = ["HardwareID"] + sorted(data_column.keys())
    sorted_names = sorted(results)
    fmt = "{:>31}"
    for i in range(len(data_column.keys())):
        fmt += "| {:>" + "{w}".format(w=len(data_column.keys()[i])) + "}"
    print(fmt.format("Battery Name", *data_column.keys()))
    for name in sorted_names:
        color = getColor(results[name])
        v = [results[name][k] if k in results[name] else "N/A" for k in data_column.keys()]
        print(color + fmt.format(name, *v) + Fore.RESET)

if __name__ == '__main__':
    init()
    rospy.init_node('battery_summary')
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, callback)

    while not rospy.is_shutdown() and keep_flag:
        print("aggregating battery info...")
        rospy.sleep(1)

    output()
