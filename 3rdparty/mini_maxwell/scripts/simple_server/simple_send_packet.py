#!/usr/bin/env python

from socket import *
import sys
import argparse
import time
import struct
parser = argparse.ArgumentParser(description='Simple socket client')
parser.add_argument("--port", default=8080, type=int)
parser.add_argument("--buffer-size", default=1024, type=int)
parser.add_argument("--size", default=256, type=int)
parser.add_argument("--rate", default=1, type=float)
parser.add_argument("--udp", action="store_true")
parser.add_argument("--ip", default="127.0.0.1")
args = parser.parse_args()

print "connecting to ", (args.ip, args.port)
if args.udp:
    server = socket(AF_INET, SOCK_DGRAM)
else:
    server = socket(AF_INET, SOCK_STREAM)
    server.connect((args.ip, args.port))
counter = 1
while True:
    packer = struct.Struct("!%ds" % args.size)
    #packer = struct.Struct("!%ds" % size)
    data = packer.pack(("%d" % (counter % 10)) * args.size)
    if not data:
        break
    print "sending", packer.size * 8, "bits"
    if args.udp:
        server.sendto(data, (args.ip, args.port))
    else:
        server.send(data)
    if not data:
        break
    print data
    counter = counter + 1
    time.sleep(1 / args.rate)

server.close()
