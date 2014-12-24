import rospy
import re
import os
def isMasterAlive():
    """
    return True if master alive and return False if
    master is not alive
    """
    try:
        # first check the host is available
        master = rospy.get_master()
        master_host = re.search('http://([a-zA-Z0-9\-_]*):', master.getUri()[2]).groups(1)[0]
        response = os.system("ping -W 10 -c 1 " + master_host + " > /dev/null")
        if response != 0:
            return False
        master.getSystemState()
        return True
    except Exception, e:
        return False
