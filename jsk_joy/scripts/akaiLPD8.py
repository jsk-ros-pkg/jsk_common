#!/usr/bin/env python

# joystick input driver for akaiLPD8 input device
# http://www.amazon.com/Akai-Professional-LPD8-Ultra-Portable-Controller/dp/B002M8EEW8
# When you use the button, set 'PAD' mode by the left side button.

import roslib
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *


BUTTON_INDICES = [40, 41, 42, 43,
                  36, 37, 38, 39]
AXIS_INDICES = [1, 2, 3, 4,
                5, 6, 7, 8]


def main():
   rospy.init_node('akailpd8_joy')
   pub = rospy.Publisher('/akailpd8/joy', Joy, latch=True)

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
                     p = True
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
