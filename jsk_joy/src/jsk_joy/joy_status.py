#!/usr/bin/env python

import rospy
import roslib

class XBoxStatus():
    def __init__(self, msg):
        if msg.buttons[6] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[7] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[9] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[10] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[2] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[1] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[7] > 0.1:
            self.up = True
        else:
            self.up = False
        if msg.axes[7] < -0.1:
            self.down = True
        else:
            self.down = False
        if msg.axes[6] > 0.1:
            self.left = True
        else:
            self.left = False
        if msg.axes[6] < -0.1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[3] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[0] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[4] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[5] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[2] < -0.5:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[5] < -0.5:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]


class PS3Status():
    def __init__(self, msg):
        # creating from sensor_msgs/Joy
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.axes[15] < 0:
            self.square = True
        else:
            self.square = False
        if msg.axes[4] < 0:
            self.up = True
        else:
            self.up = False
        if msg.axes[6] < 0:
            self.down = True
        else:
            self.down = False
        if msg.axes[7] < 0:
            self.left = True
        else:
            self.left = False
        if msg.axes[5] < 0:
            self.right = True
        else:
            self.right = False
        if msg.axes[12] < 0:
            self.triangle = True
        else:
            self.triangle = False
        if msg.axes[14] < 0:
            self.cross = True
        else:
            self.cross = False
        if msg.axes[13] < 0:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[10] < 0:
            self.L1 = True
        else:
            self.L1 = False
        if msg.axes[11] < 0:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[8] < 0:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[9] < 0:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]


class PS3WiredStatus():
    def __init__(self, msg):
        # creating from sensor_msgs/Joy
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[15] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[4] == 1:
            self.up = True
        else:
            self.up = False
        if msg.buttons[6] == 1:
            self.down = True
        else:
            self.down = False
        if msg.buttons[7] == 1:
            self.left = True
        else:
            self.left = False
        if msg.buttons[5] == 1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[12] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[14] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[13] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.buttons[10] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[11] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.buttons[8] == 1:
            self.L2 = True
        else:
            self.L2 = False
        if msg.buttons[9] == 1:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
