#!/usr/bin/env python

import SocketServer
import sys
import argparse
import socket
HOSTNAME = '0.0.0.0'

def runTCP(host, port, buffer_size):
    global BUFFER_SIZE
    BUFFER_SIZE = buffer_size
    server = SocketServer.TCPServer((host, port), Handler)
    server.serve_forever()

def printData(data):
    print data, 
    print "(%d bytes)" % sys.getsizeof(data)
    
class Handler(SocketServer.StreamRequestHandler):
    def handle(self):
        global BUFFER_SIZE
        while True:
            data = self.request.recv(BUFFER_SIZE)
            printData(data)
            if len(data) == 0:
                break
            #self.request.send(data)
        self.request.close()

def runUDP(host, port, buffer_size):
    clientsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clientsock.bind((host, port))
    while True:
        recv_msg, addr = clientsock.recvfrom(buffer_size)
        printData(recv_msg)

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simple socket server')
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--buffer-size", default=1024, type=int)
    parser.add_argument("--udp", action="store_true")
    args = parser.parse_args()
    
    if args.udp:
        print 'listening UDP', (HOSTNAME, args.port), "buffer size is", args.buffer_size
        runUDP(HOSTNAME, args.port, args.buffer_size)
    else:
        print 'listening TCP', (HOSTNAME, args.port), "buffer size is", args.buffer_size
        runTCP(HOSTNAME, args.port, args.buffer_size)
    
    
