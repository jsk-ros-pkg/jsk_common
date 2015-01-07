#!/usr/bin/env python

import SocketServer
import sys
import argparse
HOST = '127.0.0.1'
PORT = 8080

class Handler(SocketServer.StreamRequestHandler):
    def handle(self):
        global BUFFER_SIZE
        while True:
            data = self.request.recv(BUFFER_SIZE)
            print data, 
            print "(%d bytes)" % sys.getsizeof(data)
            if len(data) == 0:
                break
            self.request.send(data)
        self.request.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simple socket server')
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--buffer-size", default=1024, type=int)
    args = parser.parse_args()
    BUFFER_SIZE = args.buffer_size
    server = SocketServer.TCPServer((HOST, args.port), Handler)
    print 'listening', server.socket.getsockname(), "buffer size is", BUFFER_SIZE
    server.serve_forever()
