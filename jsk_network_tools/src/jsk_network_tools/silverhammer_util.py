from StringIO import StringIO
from struct import Struct, pack, unpack
import re

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
    
