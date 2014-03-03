#!/usr/bin/env python

import roslib
import rospy

import pygame
import pygame.joystick

import sys

from sensor_msgs.msg import *

def main():
   node_name = "simple_joy"
   if len(sys.argv) > 1:
      node_name = sys.argv[1]

   rospy.init_node(node_name)
   pub = rospy.Publisher("/" + node_name + "/joy", Joy, latch=True)

   pygame.init()
   pygame.joystick.init()
   devices = pygame.joystick.get_count()
   if devices < 1:
      rospy.logerr("No Joystick devices detected")
      exit(-1)
   rospy.loginfo("Found %d Joystick devices" % devices)

   joy_name = None
   input_dev = 0
   if len(sys.argv) > 2:
      try:
         input_dev = int(sys.argv[2])
      except ValueError:
         joy_name = sys.argv[2]
   else:
      rospy.loginfo("no input device supplied. will try to use default device.")
      input_dev = 0
   rospy.loginfo("Using input device %d" % input_dev)

   controller = None
   if joy_name == None:
      controller = pygame.joystick.Joystick(input_dev)
   else:
      for i in range(pygame.joystick.get_count()):
         if joy_name in pygame.joystick.Joystick(i).get_name():
            controller = pygame.joystick.Joystick(i)

   if controller == None:
      rospy.logerr("No Joystick controller generated")
      exit(-1)

   controller.init()
   axes = controller.get_numaxes()
   buttons = controller.get_numbuttons()
   hats = controller.get_numhats()

   rospy.loginfo("Opened it")

   m = Joy()
   m.axes = [ 0 ] * axes
   m.buttons = [ 0 ] * buttons

   p = False
   done = False

   while not rospy.is_shutdown() and done==False:
      m.header.stamp = rospy.Time.now()

      for event in pygame.event.get():
         if event.type == pygame.QUIT:
            done=True
         if event.type == pygame.JOYBUTTONDOWN:
            done=False
         if event.type == pygame.JOYBUTTONUP:
            done=False

      for i in range( axes ):
         axis = controller.get_axis( i )
         m.axes[i] = axis

      for i in range( buttons ):
         button = controller.get_button( i )
         m.buttons[i] = button

      for i in range( hats ):
         hat = controller.get_hat( i )

      pub.publish(m)
      rospy.sleep(0.01) # 100hz max

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass
