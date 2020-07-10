#!/usr/bin/env python

import hashlib
import rospy
import sys

if __name__ == "__main__":
    param_name = sys.argv[1]
    m = hashlib.md5()
    m.update(str(rospy.get_param(param_name)))
    print(m.hexdigest())
