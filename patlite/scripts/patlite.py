#!/usr/bin/env python
import sys
import socket
import binascii
from contextlib import closing

class Patlite:
    """ This is a controller for a network patlite """

    # light state
    LIGHT_OFF = '\x00'
    LIGHT_ON = '\x01'
    LIGHT_FLASH_1 = '\x02'
    LIGHT_FLASH_2 = '\x03'
    LIGHT_REMAIN = '\x09'

    # buzzer state
    BUZZER_OFF = '\x00'
    BUZZER_ON_1 = '\x01'
    BUZZER_ON_2 = '\x02'
    BUZZER_ON_3 = '\x03'
    BUZZER_ON_4 = '\x04'
    BUZZER_REMAIN = '\x00'

    # write command
    WRITE_HEADER = '\x58\x58\x53\x00\x00\x06'
    # write response
    ACK = '\x06'
    NAK = '\x15'

    # read command
    READ = '\x58\x58\x47\x00\x00\x00'

    # clear command
    CLEAR = '\x58\x58\x43\x00\x00\x00'

    def __init__(self, host, port=10000, timeout=1):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.bufsize = 512
        self.sock = None
        self.clearstate()

    def clearstate(self):
        self.state = \
            self.LIGHT_REMAIN + \
            self.LIGHT_REMAIN + \
            self.LIGHT_REMAIN + \
            self.LIGHT_REMAIN + \
            self.LIGHT_REMAIN + \
            self.BUZZER_REMAIN

    def __enter__(self):
        self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        return self.sock.connect((self.host, self.port))

    def close(self):
        return self.sock.close()

    def write(self):
        ret = None
        try:
            self.sock.sendall(self.WRITE_HEADER + self.state)
            ret = self.sock.recv(self.bufsize)
        except socket.timeout:
            print "timeout"
        if (ret != self.ACK):
            return False
        return True

    def read(self):
        try:
            self.sock.sendall(self.READ)
            tmp_state = self.sock.recv(self.bufsize)
        except socket.timeout:
            print "timeout"
            return None
        return tmp_state

    def clear(self):
        try:
            self.sock.sendall(self.CLEAR)
            ret = self.sock.recv(self.bufsize)
        except socket.timeout:
            print "timeout"
            return False
        if (ret != self.ACK):
            return False
        return True

    def red(self, value):
        self.clearstate()
        self.state = value + self.state[1:]
        return self.write()

    def yellow(self, value):
        self.clearstate()
        self.state = self.state[0] + value + self.state[2:]
        return self.write()

    def green(self, value):
        self.clearstate()
        self.state = self.state[:2] + value + self.state[3:]
        return self.write()

    def blue(self, value):
        self.clearstate()
        self.state = self.state[:3] + value + self.state[4:]
        return self.write()

    def white(self, value):
        self.clearstate()
        self.state = self.state[:4] + value + self.state[5:]
        return self.write()

    def buzzer(self, value):
        self.clearstate()
        self.state = self.state[:5] + value
        return self.write()

    def print_state(self):
        print ''.join(['%x ' % ord(s) for s in self.state])


def test():
    patlite = Patlite("10.68.0.10")
    with patlite:
        patlite.red(patlite.LIGHT_ON)
        patlite.yellow(patlite.LIGHT_ON)
        patlite.green(patlite.LIGHT_ON)

if __name__ == '__main__':
    test()

