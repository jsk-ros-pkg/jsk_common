#!/usr/bin/env python

# joystick input driver for nanoKontrol input device
# http://www.amazon.co.jp/KORG-USB-MIDI-%E3%82%B3%E3%83%B3%E3%83%88%E3%83%AD%E3%83%BC%E3%83%A9%E3%83%BC-NANO-KONTROL2/dp/B004M8UZS8

import roslib
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *


BUTTON_INDICES = [58, 59,
                  46, 60, 61, 62,
                  43, 44, 42, 41, 45,
                  32, 33, 34, 35, 36, 37, 38, 39,
                  48, 49, 50, 51, 52, 53, 54, 55,
                  64, 65, 66, 67, 68, 69, 70, 71]
AXIS_INDICES = [16, 17, 18, 19, 20, 21, 22, 23,
                0, 1, 2, 3, 4, 5, 6, 7]


def main():
   rospy.init_node('nanokontrol_joy')
   pub = rospy.Publisher('/nanokontrol/joy', Joy, latch=True)

   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      rospy.logerr("No MIDI devices detected")
      exit(-1)
   rospy.loginfo("Found %d MIDI devices" % devices)

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      rospy.loginfo("no input device supplied. will try to use default device.")
      input_dev = pygame.midi.get_default_input_id()
      if input_dev == -1:
         rospy.logerr("No default MIDI input device")
         exit(-1)
   rospy.loginfo("Using input device %d" % input_dev)

   controller = pygame.midi.Input(input_dev)
   rospy.loginfo("Opened it")

   m = Joy()
   m.axes = [ 0 ] * len(AXIS_INDICES)
   m.buttons = [ 0 ] * len(BUTTON_INDICES)

   p = False

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()
      
      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         rospy.loginfo("%s" % data)
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]
            button_index = control[1]

            for bi, i in zip(BUTTON_INDICES, range(len(BUTTON_INDICES))):
               if button_index == bi:
                  if control[2] == 127:
                     m.buttons[i] = 1
                  else:
                     m.buttons[i] = 0
               p = True
            for bi, i in zip(AXIS_INDICES, range(len(AXIS_INDICES))):
               if button_index == bi:
                  m.axes[i] = control[2] / 127.0
                  p = True
      if p:
         pub.publish(m)
         p = False

      rospy.sleep(0.01) # 100hz max
                  


if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass
