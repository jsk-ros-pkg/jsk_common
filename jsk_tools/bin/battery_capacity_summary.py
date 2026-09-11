#!/usr/bin/env python

from collections import OrderedDict
import sys
from diagnostic_msgs.msg import DiagnosticArray
try:
    from colorama import Fore, Style, init
except:
  print("Please install colorama by pip install colorama")
  sys.exit(1)

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION


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
    for key in data_column.keys():
        fmt += "| {:>" + "{w}".format(w=len(key)) + "}"
    print(fmt.format("Battery Name", *data_column.keys()))
    for name in sorted_names:
        color = getColor(results[name])
        v = [results[name][k] if k in results[name] else "N/A" for k in data_column.keys()]
        print(color + fmt.format(name, *v) + Fore.RESET)

if __name__ == '__main__':
    init()
    if ROS_VERSION == 2:
        ros_compat.rclpy.init()
        node = ros_compat.rclpy.create_node('battery_summary')
    else:
        node = None
        ros_compat.rospy.init_node('battery_summary')
    ros_compat.create_subscription(node, "/diagnostics_agg", DiagnosticArray, callback, 1)
    if ROS_VERSION == 2:
        while ros_compat.rclpy.ok() and keep_flag:
            print("aggregating battery info...")
            ros_compat.rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
        if ros_compat.rclpy.ok():
            ros_compat.rclpy.shutdown()
    else:
        while not ros_compat.rospy.is_shutdown() and keep_flag:
            print("aggregating battery info...")
            ros_compat.rospy.sleep(1)

    output()
