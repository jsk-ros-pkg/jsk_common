#!/usr/bin/env python

from socket import *
import sys
import argparse

parser = argparse.ArgumentParser(description='Simple socket client')
parser.add_argument("--port", default=8080, type=int)
parser.add_argument("--buffer-size", default=1024, type=int)
parser.add_argument("--ip", default="127.0.0.1")
args = parser.parse_args()

print "connecting to ", (args.ip, args.port)
tcpCliSock = socket(AF_INET, SOCK_STREAM)
tcpCliSock.connect((args.ip, args.port))

while True:
    data = raw_input('> ')
    if not data:
        break
    print "sending", sys.getsizeof(data), "bytes"
    tcpCliSock.send(data)
    
    if not data:
        break
    print data

tcpCliSock.close()
