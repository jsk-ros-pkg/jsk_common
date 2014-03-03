#!/usr/bin/env python

import rospy
import roslib

class TrackpointStatus():
    def __init__(self, msg):
        if msg.buttons[0] == 1:
            self.left = True
        else:
            self.left = False
        if msg.buttons[1] == 1:
            self.middle = True
        else:
            self.middle = False
        if msg.buttons[2] == 1:
            self.right = True
        else:
            self.right = False
        self.x = msg.axes[0]
        self.y = msg.axes[1]


import pprint

def joyCB(msg):
    status = TrackpointStatus(msg)
    pprint.PrettyPrinter().pprint(status.__dict__)


def main():
    import roslib
    import rospy
    from sensor_msgs.msg import Joy

    rospy.sleep(1)
    rospy.init_node('trackpoint_controller')
    s = rospy.Subscriber('/trackpoint/joy', Joy, joyCB)
  
    rospy.spin()
  
if __name__ == '__main__':
    main()


