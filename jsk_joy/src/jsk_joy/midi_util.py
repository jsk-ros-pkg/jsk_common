import pygame
import pygame.midi

class MIDIException(Exception):
  pass

def MIDIParse(message):
  """returns (type, index, value)"""
  midi_command = message[0][0]
  midi_param1 = message[0][1]
  midi_param2 = message[0][2]
  timestamp = message[1]
  command_type = MIDICommand.detect(midi_command)
  midi_channel = MIDICommand.getChannel(midi_command)
  if command_type == MIDICommand.NOTE_OFF:
    return (MIDICommand.NOTE_ON, midi_param1, 0)
  elif command_type == MIDICommand.NOTE_ON:
    return (MIDICommand.NOTE_ON, midi_param1, midi_param2 / 127.0)
  elif command_type == MIDICommand.AFTERTOUCH:
    return (MIDICommand.AFTERTOUCH, midi_param1, midi_param2 / 127.0)
  elif command_type == MIDICommand.CONTINUOUS_CONTROLLER:
    return (MIDICommand.CONTINUOUS_CONTROLLER, midi_param1, midi_param2 / 127.0)
  elif command_type == MIDICommand.PATCH_CHANGE:
    return (MIDICommand.PATCH_CHANGE, midi_param1, midi_param2 / 127.0)   #??
  elif command_type == MIDICommand.CHANNEL_PRESSURE:
    return (MIDICommand.CHANNEL_PRESSURE, midi_channel, midi_param1)
  elif command_type == MIDICommand.PITCH_BEND:
    return (MIDICommand.PITCH_BEND, midi_channel, midi_param1 / 127.0)
  else:
    raise MIDIException("unknown command type: " + MIDICommand.toStr(command_type))
    
class MIDICommand():
  NOTE_OFF = 0x80
  NOTE_ON = 0x90
  AFTERTOUCH = 0xA0
  CONTINUOUS_CONTROLLER = 0xB0
  PATCH_CHANGE = 0xC0
  CHANNEL_PRESSURE = 0xD0
  PITCH_BEND = 0xE0
  NON_MUSICAL = 0xF0
  @classmethod
  def detect(cls, val):
    types = [cls.NOTE_OFF, cls.NOTE_ON, cls.AFTERTOUCH, cls.CONTINUOUS_CONTROLLER,
             cls.PATCH_CHANGE, cls.CHANNEL_PRESSURE, cls.PITCH_BEND,
             cls.NON_MUSICAL]
    for t in types:
      if cls.checkUpperByte(t, val):
        return t
    raise MIDIException("cannot detect the type of " + str(val))
  @classmethod
  def allCommands(cls):
    return [cls.NOTE_OFF, cls.NOTE_ON, cls.AFTERTOUCH, cls.CONTINUOUS_CONTROLLER,
            cls.PATCH_CHANGE, cls.CHANNEL_PRESSURE, cls.PITCH_BEND, 
            cls.NON_MUSICAL]
  @classmethod
  def checkUpperByte(cls, ref, val):
    return (ref >> 4) == (val >> 4)
  @classmethod
  def getChannel(cls, val):
    return (0x0F & val)
  @classmethod
  def toStr(cls, val):
    if val == cls.NOTE_OFF:
      return "Note-off"
    elif val == cls.NOTE_ON:
      return "Note-on"
    elif val == cls.AFTERTOUCH:
      return "Aftertouch"
    elif val == cls.CONTINUOUS_CONTROLLER:
      return "ContinuousController"
    elif val == cls.PATCH_CHANGE:
      return "Patch Change"
    elif val == cls.CHANNEL_PRESSURE:
      return "Channel Pressure"
    elif val == cls.PITCH_BEND:
      return "Pitch Bend"
    elif val == cls.NON_MUSICAL:
      return "Non Musical Command"
    else:
      raise MIDIException("Unknown command: " + str(val))
    
def openMIDIInputByName(device_name):
  return openMIDIByName(device_name, 1)
  
def openMIDIOutputByName(device_name):
  return openMIDIByName(device_name, 0)
  
def openMIDIByName(device_name, input_output):
  devices = pygame.midi.get_count()
  for i in range(devices):
    info = pygame.midi.get_device_info(i)
    if info[1] == device_name and info[2] == input_output:
      if input_output == 1:
        return pygame.midi.Input(i)
      else:
        return pygame.midi.Output(i)
  raise MIDIException("Cannot find the device: %s" % (device_name))
  
