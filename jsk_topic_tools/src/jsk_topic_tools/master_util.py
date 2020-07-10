# -*- coding: utf-8 -*-

import urlparse
import rospy
import os

previous_run_id = None


def isMasterAlive(timeout_sec, trials):
    """
    return True if master alive and return False if
    master is not alive
    """
    global previous_run_id
    try:
        # first check the host is available
        master_host = urlparse.urlsplit(rospy.core.rosgraph.get_master_uri()).hostname
        for i in range(trials):
            response = os.system("ping -W {} -c 1 {} > /dev/null".format(timeout_sec, master_host))
            if response == 0:
                # host machine is available, immediately break from for loop
                break
            # Ping failed trials times in a row, the master computer is regarded as dead.
            if i == trials - 1:
                print("master machine looks down because ping fails with timeout={} {} times".format(timeout_sec, trials))
                return False
            else:
                print("ping fails {}/{} times".format(i, trials))

        assert 1 == rospy.get_master().getSystemState()[0]
        run_id = rospy.get_param("/run_id")

        if not previous_run_id:
            previous_run_id = run_id
        if run_id != previous_run_id:
            print("run_id is not same")
            previous_run_id = run_id
            return False
        return True
    except Exception as e:
        return False
