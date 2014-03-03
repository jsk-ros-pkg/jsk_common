#!/usr/bin/env python

import rospy
import roslib

class NanoKONTROL2Status():
    def __init__(self, msg):
        if msg.buttons[0] == 1:
            self.track_left = True
        else:
            self.track_left = False
        if msg.buttons[1] == 1:
            self.track_right = True
        else:
            self.track_right = False
        if msg.buttons[2] == 1:
            self.cycle = True
        else:
            self.cycle = False
        if msg.buttons[3] == 1:
            self.marker_set = True
        else:
            self.marker_set = False
        if msg.buttons[4] == 1:
            self.marker_left = True
        else:
            self.marker_left = False
        if msg.buttons[5] == 1:
            self.marker_right = True
        else:
            self.marker_right = False
        if msg.buttons[6] == 1:
            self.prev = True
        else:
            self.prev = False
        if msg.buttons[7] == 1:
            self.next = True
        else:
            self.next = False
        if msg.buttons[8] == 1:
            self.pause = True
        else:
            self.pause = False
        if msg.buttons[9] == 1:
            self.play = True
        else:
            self.play = False
        if msg.buttons[10] == 1:
            self.rec = True
        else:
            self.rec = False

        if msg.buttons[11] == 1:
            self.S1 = True
        else:
            self.S1 = False
        if msg.buttons[12] == 1:
            self.S2 = True
        else:
            self.S2 = False
        if msg.buttons[13] == 1:
            self.S3 = True
        else:
            self.S3 = False
        if msg.buttons[14] == 1:
            self.S4 = True
        else:
            self.S4 = False
        if msg.buttons[15] == 1:
            self.S5 = True
        else:
            self.S5 = False
        if msg.buttons[16] == 1:
            self.S6 = True
        else:
            self.S6 = False
        if msg.buttons[17] == 1:
            self.S7 = True
        else:
            self.S7 = False
        if msg.buttons[18] == 1:
            self.S8 = True
        else:
            self.S8 = False
        if msg.buttons[19] == 1:
            self.M1 = True
        else:
            self.M1 = False
        if msg.buttons[20] == 1:
            self.M2 = True
        else:
            self.M2 = False
        if msg.buttons[21] == 1:
            self.M3 = True
        else:
            self.M3 = False
        if msg.buttons[22] == 1:
            self.M4 = True
        else:
            self.M4 = False
        if msg.buttons[23] == 1:
            self.M5 = True
        else:
            self.M5 = False
        if msg.buttons[24] == 1:
            self.M6 = True
        else:
            self.M6 = False
        if msg.buttons[25] == 1:
            self.M7 = True
        else:
            self.M7 = False
        if msg.buttons[26] == 1:
            self.M8 = True
        else:
            self.M8 = False
        if msg.buttons[27] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.buttons[28] == 1:
            self.R2 = True
        else:
            self.R2 = False
        if msg.buttons[29] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[30] == 1:
            self.R4 = True
        else:
            self.R4 = False
        if msg.buttons[31] == 1:
            self.R5 = True
        else:
            self.R5 = False
        if msg.buttons[32] == 1:
            self.R6 = True
        else:
            self.R6 = False
        if msg.buttons[33] == 1:
            self.R7 = True
        else:
            self.R7 = False
        if msg.buttons[34] == 1:
            self.R8 = True
        else:
            self.R8 = False

        self.rotate1 = msg.axes[0]
        self.rotate2 = msg.axes[1]
        self.rotate3 = msg.axes[2]
        self.rotate4 = msg.axes[3]
        self.rotate5 = msg.axes[4]
        self.rotate6 = msg.axes[5]
        self.rotate7 = msg.axes[6]
        self.rotate8 = msg.axes[7]

        self.slide1 = msg.axes[8]
        self.slide2 = msg.axes[9]
        self.slide3 = msg.axes[10]
        self.slide4 = msg.axes[11]
        self.slide5 = msg.axes[12]
        self.slide6 = msg.axes[13]
        self.slide7 = msg.axes[14]
        self.slide8 = msg.axes[15]



import pprint

def joyCB(msg):
    status = NanoKONTROL2Status(msg)
    pprint.PrettyPrinter().pprint(status.__dict__)


def main():
    import roslib
    import rospy
    from sensor_msgs.msg import Joy

    rospy.sleep(1)
    rospy.init_node('nanokontrol_controller')
    s = rospy.Subscriber('/nanokontrol/joy', Joy, joyCB)
  
    rospy.spin()
  
if __name__ == '__main__':
    main()


