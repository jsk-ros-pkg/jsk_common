#!/usr/bin/env python
import pygame
import pygame.midi
import select
import sys
import yaml
import rospy
import roslib

from sensor_msgs.msg import Joy, JoyFeedbackArray

roslib.load_manifest('jsk_joy')

from jsk_joy.midi_util import MIDIParse, MIDICommand, MIDIException, openMIDIInputByName, openMIDIOutputByName

def feedback_array_cb(out_controller, config, msg_arr):
  output_config = config["output"]
  for msg in msg_arr.array:
    if len(output_config) <= msg.id:
      rospy.logerr("%d is out of output configuration (%d configurations)" % (msg.id, len(output_config)))
      return
    the_config = output_config[msg.id]
    command = the_config[0]
    channel = the_config[1]
    val = int(msg.intensity * 127)
    if val < 0:
      val = 0
    elif val > 127:
      val = 127
    if the_config[2]:
      out_controller.write_short(command | channel, val, 0)
    else:
      param1 = the_config[3]
      out_controller.write_short(command | channel, param1, val)
    

def main():
  pygame.midi.init()
  rospy.init_node('midi_joy')
  # parse the arg
  argv = rospy.myargv()
  if len(argv) == 0:
    rospy.logfatal("You need to specify config yaml file")
    sys.exit(1)
  config_file = argv[1]
  joy_pub = rospy.Publisher("/joy", Joy)
  with open(config_file, "r") as f:
    config = yaml.load(f)
    # open the device
    controller = openMIDIInputByName(config["device_name"])
    
    joy = Joy()
    joy.axes = [0.0] * len(config["analogs"])
    # automatically mapping to buttons from axes if it has NOTE_OFF or NOTE_ON MIDI commands
    button_configs = [c for c in config["analogs"]
                      if c[0] == MIDICommand.NOTE_ON or c[0] == MIDICommand.NOTE_OFF]
    if config.has_key("output"):
      out_controller = openMIDIOutputByName(config["device_name"])
      s = rospy.Subscriber("~set_feedback", JoyFeedbackArray, lambda msg: feedback_array_cb(out_controller, config, msg))
    joy.buttons = [0] * len(button_configs)
    while not rospy.is_shutdown():
      joy.header.stamp = rospy.Time.now()
      while controller.poll():
        data = controller.read(1)
        for elem_set in data:
          (command, ind, val) = MIDIParse(elem_set)
          try:
            index = config["analogs"].index((command, ind))
            joy.axes[index] = val
            if command == MIDICommand.NOTE_ON or command == MIDICommand.NOTE_OFF:
              button_index = button_configs.index((command, ind))
              if val == 0.0:
                joy.buttons[button_index] = 0
              else:
                joy.buttons[button_index] = 1
          except:
            rospy.logwarn("unknown MIDI message: (%d, %d, %f)" % (command, ind, val))
      joy_pub.publish(joy)
      rospy.sleep(1.0 / 100.0)
if __name__ == '__main__':
  main()
  
