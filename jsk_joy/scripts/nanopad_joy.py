#!/usr/bin/env python

# joystick input driver for nanoPad2 input device
# http://www.amazon.co.jp/KORG-USB-MIDI-%E3%82%B3%E3%83%B3%E3%83%88%E3%83%AD%E3%83%BC%E3%83%A9%E3%83%BC-NANOPAD2-%E3%83%96%E3%83%A9%E3%83%83%E3%82%AF/dp/B004M8YPKM

import roslib
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *


BUTTON_INDICES = [37, 39, 41, 43, 45, 47, 49, 51,
                  36, 38, 40, 42, 44, 46, 48, 50]
AXIS_INDICES = []


def main():
   rospy.init_node('nanopad_joy')
   pub = rospy.Publisher('/nanopad/joy', Joy, latch=True)

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
                  if control[0] == 144:
                     m.buttons[i] = 1
                  elif control[0] == 128:
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
