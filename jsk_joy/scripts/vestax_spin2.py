#!/usr/bin/env python

# joystick input driver for nanoKontrol input device
# http://www.amazon.co.jp/KORG-USB-MIDI-%E3%82%B3%E3%83%B3%E3%83%88%E3%83%AD%E3%83%BC%E3%83%A9%E3%83%BC-NANO-KONTROL2/dp/B004M8UZS8

import roslib;
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

BUTTON_INDICES0 = range(32, 38)
BUTTON_INDICES1 = [46, 47, 48, 49, 42, 43, 44, 98, 99, 100, 101, 102, 103, 35, 36, 37, 34, 104, 45, 50, 32, 33]
SCRATCH_INDICES = [16, 17]
AXIS_INDICES0 = [16]
AXIS_INDICES1 = [28, 27, 17, 18, 19, 20, 26, 16, 29]
AXIS_INDICES2 = [22, 17, 28, 19, 18, 21, 27, 20, 23]



def main():
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      print "No MIDI devices detected"
      exit(-1)
   print "Found %d MIDI devices" % devices

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
       for i in range(devices):
           info = pygame.midi.get_device_info(i)
           if 'Vestax Spin 2 MIDI' in info[1]:
               if info[2] == 1: # is input device?
                   input_dev = i
                   break
       if not input_dev:
           print "No MIDI device"
           exit(-1)
   print "Using input device %d" % input_dev

   controller = pygame.midi.Input(input_dev)
   print "Opened it"

   rospy.init_node('kontrol')
   pub = rospy.Publisher('/joy_pad', Joy, latch=True)

   m = Joy()
   m.axes = [ 0 ] * (2 + len(AXIS_INDICES0) + len(AXIS_INDICES1) + len(AXIS_INDICES2))
   m.buttons = [ 0 ] * (len(BUTTON_INDICES0) + 2 * len(BUTTON_INDICES1))
   mode = None

   p = False

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()

      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         print data
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]
            # look for continuous controller commands

            # button
            if control[0] in [144,145,146]:
                player = control[0] - 144
                button_index = control[1]
                pressed = True if control[2] == 127 else False
                if player == 2: # common
                    for bi, i in zip(BUTTON_INDICES0, range(len(BUTTON_INDICES0))):
                        if button_index == bi:
                            if pressed:
                                m.buttons[i] = 1
                            else:
                                m.buttons[i] = 0
                            p = True
                else:
                    for bi, i in zip(BUTTON_INDICES1, range(len(BUTTON_INDICES1))):
                        if button_index == bi:
                            offset = player * len(BUTTON_INDICES1) + len(BUTTON_INDICES0)
                            if pressed:
                                m.buttons[i + offset] = 1
                            else:
                                m.buttons[i + offset] = 0
                            p = True
            # slider
            elif control[0] in [176, 177, 180]:
                player = control[0] - 176
                axis_index = control[1]

                if player == 4: # common
                    for bi, i in zip(AXIS_INDICES0, range(len(AXIS_INDICES0))):
                        if axis_index == bi:
                            offset = len(SCRATCH_INDICES)
                            m.axes[i + offset] = control[2] / 127.0
                            p = True
                elif player == 0:
                    for bi, i in zip(AXIS_INDICES1, range(len(AXIS_INDICES1))):
                        if axis_index == bi:
                            offset = len(SCRATCH_INDICES) + len(AXIS_INDICES0)
                            m.axes[i + offset] = control[2] / 127.0
                            p = True
                elif player == 1:
                    for bi, i in zip(AXIS_INDICES2, range(len(AXIS_INDICES2))):
                        if axis_index == bi:
                            offset = len(SCRATCH_INDICES) + len(AXIS_INDICES0) + len(AXIS_INDICES1)
                            m.axes[i + offset] = control[2] / 127.0
                            p = True
            # scratch
            elif control[0] == 178:
                for s, i in zip(SCRATCH_INDICES, range(len(SCRATCH_INDICES))):
                    if control[1] == s:
                        if control[2] > 64:
                            m.axes[i] = (control[2] - 65) / 15.0
                        else:
                            m.axes[i] = (control[2] - 63) / 15.0
                        p = True
            else:
               continue
      if p:
         pub.publish(m)
         p = False

      rospy.sleep(0.01) # 100hz max



if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
