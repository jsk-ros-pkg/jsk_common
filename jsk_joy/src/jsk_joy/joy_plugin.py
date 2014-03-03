# joy_plugins.py

import rospy

class JSKJoyPlugin():
    def __init__(self, name):
        self.name = name
    def joyCB(self, status):
        # a callback function
        rospy.logerr("%s: no joyCB is overriden" % (self.name))
    def enable(self):
        pass
    def disable(self):
        pass
