from jsk_joy.joy_plugin import JSKJoyPlugin

import rospy

class VerboseStatus(JSKJoyPlugin):
  def __init__(self):
    JSKJoyPlugin.__init__(self, 'VerbosePlugin')
  def joyCB(self, status, history):
    rospy.loginfo('analog left (%f, %f)' % (status.left_analog_x, status.left_analog_y))

    rospy.loginfo('analog right (%f, %f)' % (status.right_analog_x, status.right_analog_y))
    if status.select:
      rospy.loginfo('select')
    if status.start:
      rospy.loginfo('start')
    if status.L1:
      rospy.loginfo('L1')
    if status.R1:
      rospy.loginfo('R1')
    if status.L2:
      rospy.loginfo('L2')
    if status.R2:
      rospy.loginfo('R2')
    if status.L3:
      rospy.loginfo('L3')
    if status.R3:
      rospy.loginfo('R3')
    if status.up:
      rospy.loginfo('up')
    if status.down:
      rospy.loginfo('down')
    if status.left:
      rospy.loginfo('left')
    if status.right:
      rospy.loginfo('right')
    if status.triangle:
      rospy.loginfo('triangle')
    if status.circle:
      rospy.loginfo('circle')
    if status.square:
      rospy.loginfo('square')
    if status.cross:
      rospy.loginfo('cross')
    

      
