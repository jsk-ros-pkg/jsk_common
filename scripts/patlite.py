#!/usr/bin/env python
import sys
import socket
import binascii
from contextlib import closing

class PatliteState(object):
    """ This is a patlite state class """

    # target
    class Target(object):
        LIGHT_RED = 0
        LIGHT_YELLOW = 1
        LIGHT_GREEN = 2
        LIGHT_BLUE = 3
        LIGHT_WHITE = 4
        BUZZER = 5
        __slots__ = []

    # light state
    class LightState(object):
        OFF = '\x00'
        ON = '\x01'
        FLASH_1 = '\x02'
        FLASH_2 = '\x03'
        REMAIN = '\x09'
        __slots__ = []

    # buzzer state
    class BuzzerState(object):
        OFF = '\x00'
        ON_1 = '\x01'
        ON_2 = '\x02'
        ON_3 = '\x03'
        ON_4 = '\x04'
        REMAIN = '\x09'
        __slots__ = []

    def int2code(self, i):
        if i == 0: return '\x00'
        elif i == 1: return '\x01'
        elif i == 2: return '\x02'
        elif i == 3: return '\x03'
        elif i == 4: return '\x04'
        else: return '\x09'

    def code2int(self, c):
        if c == '\x00': return 0
        elif c == '\x01': return 1
        elif c == '\x02': return 2
        elif c == '\x03': return 3
        elif c == '\x04':return 4
        else: return 9

    def clear(self):
        self.state = \
            self.LightState.REMAIN + \
            self.LightState.REMAIN + \
            self.LightState.REMAIN + \
            self.LightState.REMAIN + \
            self.LightState.REMAIN + \
            self.BuzzerState.REMAIN

    def __init__(self, state=None):
        if state is None:
            self.clear()
        else:
            self.state = state

    def red(self, value):
        self.state = value + self.state[1:]
        return self.state

    def yellow(self, value):
        self.state = self.state[0] + value + self.state[2:]
        return self.state

    def green(self, value):
        self.state = self.state[:2] + value + self.state[3:]
        return self.state

    def blue(self, value):
        self.state = self.state[:3] + value + self.state[4:]
        return self.state

    def white(self, value):
        self.state = self.state[:4] + value + self.state[5:]
        return self.state

    def buzzer(self, value):
        self.state = self.state[:5] + value
        return self.state

    def set_from_int(self, target, state):
        if not self.is_valid(target, self.int2code(state)):
            raise
        self.state = self.state[:target] + self.int2code(state) + self.state[target+1:]

    def is_valid(self, target, state):
        # TODO to be implemented
        return True

    def __repr__(self):
        return "<PatliteState '%s'>" % (''.join(['%x ' % ord(s) for s in self.state]))



class Patlite(object):
    """ This is a controller for a network patlite """

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
        self.state = PatliteState()

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

    def write(self, state=None):
        ret = None
        if state is None:
            state = self.state
        try:
            self.sock.sendall(self.WRITE_HEADER + state.state)
            ret = self.sock.recv(self.bufsize)
        except socket.timeout:
            print "timeout"
        if (ret != self.ACK):
            return False
        return True

    def read(self):
        try:
            self.sock.sendall(self.READ)
            tmp_state = PatliteState(self.sock.recv(self.bufsize))
        except socket.timeout:
            print "timeout"
            return None
        return tmp_state

    def clear(self):
        ret = None
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
        self.state.clear()
        self.state.red(value)
        return self.write()

    def yellow(self, value):
        self.state.clear()
        self.state.yellow(value)
        return self.write()

    def green(self, value):
        self.state.clear()
        self.state.green(value)
        return self.write()

    def blue(self, value):
        self.state.clear()
        self.state.blue(value)
        return self.write()

    def white(self, value):
        self.state.clear()
        self.state.white(value)
        return self.write()

    def buzzer(self, value):
        self.state.clear()
        self.state.buzzer(value)
        return self.write()


def test():
    patlite = Patlite("10.68.0.10")
    with patlite:
        patlite.red(patlite.state.LightState.ON)
        patlite.yellow(patlite.state.LightState.ON)
        patlite.green(patlite.state.LightState.ON)

if __name__ == '__main__':
    test()

