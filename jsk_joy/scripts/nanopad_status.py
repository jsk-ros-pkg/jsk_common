#!/usr/bin/env python

import rospy
import roslib

class NanoPAD2Status():
    def __init__(self, msg):
        if msg.buttons[0] == 1:
            self.buttonU1 = True
        else:
            self.buttonU1 = False
        if msg.buttons[1] == 1:
            self.buttonU2 = True
        else:
            self.buttonU2 = False
        if msg.buttons[2] == 1:
            self.buttonU3 = True
        else:
            self.buttonU3 = False
        if msg.buttons[3] == 1:
            self.buttonU4 = True
        else:
            self.buttonU4 = False
        if msg.buttons[4] == 1:
            self.buttonU5 = True
        else:
            self.buttonU5 = False
        if msg.buttons[5] == 1:
            self.buttonU6 = True
        else:
            self.buttonU6 = False
        if msg.buttons[6] == 1:
            self.buttonU7 = True
        else:
            self.buttonU7 = False
        if msg.buttons[7] == 1:
            self.buttonU8 = True
        else:
            self.buttonU8 = False

        if msg.buttons[8] == 1:
            self.buttonL1 = True
        else:
            self.buttonL1 = False
        if msg.buttons[9] == 1:
            self.buttonL2 = True
        else:
            self.buttonL2 = False
        if msg.buttons[10] == 1:
            self.buttonL3 = True
        else:
            self.buttonL3 = False
        if msg.buttons[11] == 1:
            self.buttonL4 = True
        else:
            self.buttonL4 = False
        if msg.buttons[12] == 1:
            self.buttonL5 = True
        else:
            self.buttonL5 = False
        if msg.buttons[13] == 1:
            self.buttonL6 = True
        else:
            self.buttonL6 = False
        if msg.buttons[14] == 1:
            self.buttonL7 = True
        else:
            self.buttonL7 = False
        if msg.buttons[15] == 1:
            self.buttonL8 = True
        else:
            self.buttonL8 = False



import pprint

def joyCB(msg):
    status = NanoPAD2Status(msg)
    pprint.PrettyPrinter().pprint(status.__dict__)


def main():
    import roslib
    import rospy
    from sensor_msgs.msg import Joy

    rospy.sleep(1)
    rospy.init_node('nanopad_controller')
    s = rospy.Subscriber('/nanopad/joy', Joy, joyCB)
  
    rospy.spin()
  
if __name__ == '__main__':
    main()


