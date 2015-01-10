from StringIO import StringIO
from struct import Struct, pack, unpack
import re
import rospy
import roslib
import roslib.message

# utility function
def parseMessageType(field_string):
    match = re.match("(.*)\[([\d]*)\]", field_string)
    if match:
        num_string = match.group(2)
        return (match.group(1), int(num_string))
    else:
        return (field_string, 1)
def fieldToTopic(field):
    field.replace("__", "/")
    
def msgToStructFormat(msg):
    slots = list(msg.__slots__)           #copy
    slot_types = list(msg._slot_types)
    fmt_stream = StringIO()
    fmt_stream.write("!")
    for slot, slot_type in zip(slots, slot_types):
        parsed_type = parseMessageType(slot_type)
        field_type = parsed_type[0]
        field_length = parsed_type[1]
        if field_type == "char":
            fmt_stream.write("c" * field_length)
        elif field_type == "bool":
            fmt_stream.write("?" * field_length)
        elif field_type == "int8":
            fmt_stream.write("b" * field_length)
        elif field_type == "uint8":
            fmt_stream.write("B" * field_length)
        elif field_type == "int16":
            raise Exception("int16 is not supported")
        elif field_type == "uint16":
            fmt_stream.write("H" * field_length)
        elif field_type == "int32":
            fmt_stream.write("i" * field_length)
        elif field_type == "uint32":
            fmt_stream.write("I" * field_length)
        elif field_type == "int64":
            fmt_stream.write("q" * field_length)
        elif field_type == "uint64":
            fmt_stream.write("Q" * field_length)
        elif field_type == "float32":
            fmt_stream.write("f" * field_length)
        elif field_type == "float64":
            fmt_stream.write("d" * field_length)
        elif field_type == "string":
            raise Excception("string is not supported!, please use char[static_length]")
        elif field_type == "duration" or field_type == "time":
            raise Excception("duration and time are not supported")
    return fmt_stream.getvalue()

def packableValue(value, value_type):
    if value_type == "bool":
        return value
    else:
        return ord(value)
    
def packMessage(msg, fmt):
    data = []
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        slot_value = getattr(msg, slot)
        parsed_type = parseMessageType(slot_type)
        field_type = parsed_type[0]
        if hasattr(slot_value, "__len__"):   #array
            for i in range(len(slot_value)):
                data.append(packableValue(slot_value[i], field_type))
        else:
            data.append(packableValue(slot_value, field_type))
    packed = pack(fmt, *data)
    return packed

def unpackArrayValue(array, field_type):
    if field_type == "bool":
        return [ord(v) for v in value]
    else:
        return array

def unpackValue(val, field_type):
    if field_type == "bool":
        return ord(val) == 1
    else:
        return val
    
def unpackMessage(data, fmt, message_class):
    unpacked_data = unpack(fmt, data)
    msg = message_class()
    counter = 0
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        slot_value = getattr(msg, slot)
        parsed_type = parseMessageType(slot_type)
        field_type = parsed_type[0]
        field_length = parsed_type[1]
        target_data = data[counter:counter + field_length]
        if hasattr(slot_value, "__len__"):   #array
            setattr(msg, slot, unpackArrayValue(target_data, field_type))
        else:
            setattr(msg, slot, unpackValue(target_data[0], field_type))
        counter = counter + field_length
    return msg

def publishersFromMessage(msg, prefix=""):
    ret = []
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        topic_name = prefix + "/" + slot.replace("__", "/")
        try:
            msg_class = roslib.message.get_message_class(slot_type)
        except:
            raise Exception("invalid topic type: %s"%slot_type)
        ret.append(rospy.Publisher(topic_name, msg_class))
    return ret

def decomposeLargeMessage(msg, prefix=""):
    ret = dict()
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        topic_name = prefix + "/" + slot.replace("__", "/")
        print topic_name
        ret[topic_name] = getattr(msg, slot)
    return ret
        
    
def subscribersFromMessage(msg):
    ret = []
    for slot, slot_type in zip(msg.__slots__, msg._slot_types):
        topic_name = "/" + slot.replace("__", "/")
        try:
            msg_class = roslib.message.get_message_class(slot_type)
        except:
            raise Exception("invalid topic type: %s"%slot_type)
        ret.append((topic_name, msg_class))
    return ret

#################################################w
# Packet definition for Large Data
# |SeqID(4byte)|ID(4byte)|NUM_PACKET(4byte)|data.... |
#################################################w
class LargeDataUDPPacket():
    def __init__(self, seq_id, id, num, data, packet_size):
        self.id = id
        self.seq_id = seq_id
        self.num = num
        self.data = data
        self.packet_size = packet_size
    def pack(self):
        return pack("!III%ds" % (len(self.data)),
                    self.seq_id, self.id, self.num, self.data)
    #def pack(self):
    @classmethod
    def headerSize(cls):
        return 4 + 4 + 4
    @classmethod
    def fromData(cls, data, packet_size):
        unpacked = unpack("!III%ds" % (len(data) - cls.headerSize()), data)
        ret = cls(unpacked[0], unpacked[1], unpacked[2], unpacked[3], 
                  packet_size)
        #print "buffer length", len(unpacked[3])
        return ret
        
def separateBufferIntoPackets(seq_id, buffer, packet_size):
    buffer_packet_size = packet_size - LargeDataUDPPacket.headerSize()
    num_packet = len(buffer) / buffer_packet_size
    if len(buffer) % buffer_packet_size != 0:
        num_packet = num_packet + 1
    packets = []
    for i in range(num_packet):
        packets.append(LargeDataUDPPacket(seq_id, i, num_packet, 
                                          buffer[i*buffer_packet_size:(i+1)*buffer_packet_size],
                                          packet_size))
        #print "buffer length", len(buffer[i*num_packet:(i+1)*num_packet])
    return packets
