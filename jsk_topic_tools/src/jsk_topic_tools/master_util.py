# -*- coding: utf-8 -*-

import urlparse
import rospy
import os

previous_run_id = None


def isMasterAlive():
    """
    return True if master alive and return False if
    master is not alive
    """
    global previous_run_id
    try:
        # first check the host is available
        master_host = urlparse.urlsplit(rospy.core.rosgraph.get_master_uri()).hostname
        response = os.system("ping -W 10 -c 1 " + master_host + " > /dev/null")
        if response != 0:
            print "master machine looks down"
            return False
        assert 1 == rospy.get_master().getSystemState()[0]
        run_id = rospy.get_param("/run_id")

        if not previous_run_id:
            previous_run_id = run_id
        if run_id != previous_run_id:
            print "run_id is not same"
            previous_run_id = run_id
            return False
        return True
    except Exception, e:
        return False
