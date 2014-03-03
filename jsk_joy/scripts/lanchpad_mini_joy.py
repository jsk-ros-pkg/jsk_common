#!/usr/bin/env python

# joystick input driver for Launchpad Mini input device
# http://www.amazon.com/Novation-Launchpad-Controller-Performing-Producing/dp/B00F9SWK9M

import roslib
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *


BUTTON_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 
                  16, 17, 18, 19, 20, 21, 22, 23,
                  32, 33, 34, 35, 36, 37, 38, 39,
                  48, 49, 50, 51, 52, 53, 54, 55,
                  64, 65, 66, 67, 68, 69, 70, 71,
                  80, 81, 82, 83, 84, 85, 86, 87,
                  96, 97, 98, 99, 100, 101, 102, 103,
                  112, 113, 114, 115, 116, 117, 118, 119,
                  104, 105, 106, 107, 108, 109, 110, 111,
                  8, 24, 40, 56, 72, 88, 104, 120
                  ]

AXIS_INDICES = []
COLOR_R = 3
COLOR_G = 3

def main():
   rospy.init_node('launchpad_mini_joy')
   pub = rospy.Publisher('/launchpad_mini/joy', Joy, latch=True)

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

   if len(sys.argv) > 2:
      output_dev = int(sys.argv[2])
   else:
      rospy.loginfo("no output device supplied. will try to use default device.")
      output_dev = pygame.midi.get_default_output_id()

      if output_dev == -1:
         rospy.logerr("No default MIDI output device")
         exit(-1)

   rospy.loginfo("Using input device %d" % input_dev)
   rospy.loginfo("Using output device %d" % output_dev)

   controller = pygame.midi.Input(input_dev)
   midi_output = pygame.midi.Output(output_dev)
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
            button_type = control[0]
            button_index = control[1]

            for bi, i in zip(BUTTON_INDICES, range(len(BUTTON_INDICES))):
               if button_index == bi:
                  #duplicated number
                  if i == 64 and button_type != 176:
                     continue
                  if i == 78 and button_type != 144:
                     continue

                  if control[2] == 127:
                     m.buttons[i] = 1
                     midi_output.write([[[button_type ,button_index , COLOR_G * 4 + COLOR_R ], timestamp]])
                  else:
                     m.buttons[i] = 0
                     midi_output.write([[[button_type ,button_index , 0], timestamp]])
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
