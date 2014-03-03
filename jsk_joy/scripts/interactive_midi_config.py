#!/usr/bin/env python

import pygame
import pygame.midi
import select
import sys
import yaml
import roslib

roslib.load_manifest("jsk_joy")

from jsk_joy.midi_util import MIDIParse, MIDICommand, MIDIException

G_DEVICE_INFO = {
  "device_name": "",
  "analogs": [] #[[command, index], [command, index], ...
  }

class ParseException(Exception):
  pass

def parseDeviceName():
  global G_DEVICE_INFO
  devices = pygame.midi.get_count()
  print "==========================================="
  print "First, we choose device name:"
  for d in range(devices):
    info = pygame.midi.get_device_info(d)
    if info[2] == 1:
      print "  [%d] %s (%s)" % (d, info[1], "input")
    else:
      print "  [%d] %s (%s)" % (d, info[1], "output")
  val = raw_input("Please select the device by number[%d-%d]:" % (0, d))
  try:
    parsed_number = int(val)
    if parsed_number >= 0 and parsed_number <= d:
      name = pygame.midi.get_device_info(parsed_number)[1]
      G_DEVICE_INFO["device_name"] = name
      print ""
      print "device_name: %s"  % (name)
      return parsed_number
    else:
      raise ParseException("please input number bewtween %d to %d" % (0, d))
  except ValueError:
    raise ParseException("please input number")

def configAnalogInputs(controller):
  global G_DEVICE_INFO
  print "==========================================="
  print "Please move ALL the inputs"
  print "==========================================="
  print "The order you move them will be mapped into Joy/axes."
  print "If you want to finish analog mapping, please type 'q'"
  analog_configs = []
  while True:
    ready = select.select([sys.stdin], [], [], 0.1)[0]
    if ready:
      line = sys.stdin.readline()
      if line.startswith("q"):
        print "We installed %d analog inputs" % (len(analog_configs))
        G_DEVICE_INFO["analogs"] = analog_configs
        return
    while controller.poll():
      data = controller.read(1)
      for elem_set in data:
        try:
          (command, index, val) = MIDIParse(elem_set)
          if (command, index) not in analog_configs:
            print "(%d, %d) installing into %d" % (command, index, len(analog_configs))
            analog_configs.append((command, index))
        except MIDIException, e:
          print "(%d, %d, %d) is not supported" % (elem_set[0][0], elem_set[0][1], elem_set[0][2])

def main():
  pygame.midi.init()
  while True:
    try:
      device_num = parseDeviceName()
      break
    except ParseException, e:
      print e.message
      print ""
      continue
  controller = pygame.midi.Input(device_num)
  configAnalogInputs(controller)
  f = open('/tmp/midi.yaml', 'w')
  f.write(yaml.dump(G_DEVICE_INFO))
  f.close()
  print "writing the configuration to /tmp/midi.yaml"
  
if __name__ == "__main__":
  main()

  
