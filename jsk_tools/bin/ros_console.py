#!/usr/bin/env python
import rospy

import sys
import colorama
from colorama import Fore, Style
import argparse
from rosgraph_msgs.msg import Log
from threading import Lock
import math

def levelGreaterEqualThan(msg, level):
    return msg.level >= levelValue(level)

def levelValue(level):
    if level == "DEBUG":
        return Log.DEBUG
    elif level == "INFO":
        return Log.INFO
    elif level == "WARN":
        return Log.WARN
    elif level == "ERROR":
        return Log.ERROR
    elif level == "FATAL":
        return Log.FATAL
    
def levelString(msg):
    if msg.level == Log.DEBUG:
        return "[DEBUG]"
    elif msg.level == Log.INFO:
        return "[INFO]"
    elif msg.level == Log.WARN:
        return "[WARN]"
    elif msg.level == Log.ERROR:
        return "[ERROR]"
    elif msg.level == Log.FATAL:
        return "[FATAL]"

def coloredMessage(msg):
    if msg.level == Log.DEBUG:
        return Fore.GREEN + msg.msg
    elif msg.level == Log.INFO:
        return msg.msg
    elif msg.level == Log.WARN:
        return Fore.YELLOW + msg.msg
    elif msg.level == Log.ERROR:
        return Fore.RED + msg.msg
    elif msg.level == Log.FATAL:
        return Fore.RED + msg.msg

def stampString(msg):
    return "[{:10.2f}]".format(msg.header.stamp.to_sec())

# from http://stackoverflow.com/questions/566746/how-to-get-console-window-width-in-python
def getTerminalSize():
    import os
    env = os.environ
    def ioctl_GWINSZ(fd):
        try:
            import fcntl, termios, struct, os
            cr = struct.unpack('hh', fcntl.ioctl(fd, termios.TIOCGWINSZ,
        '1234'))
        except:
            return
        return cr
    cr = ioctl_GWINSZ(0) or ioctl_GWINSZ(1) or ioctl_GWINSZ(2)
    if not cr:
        try:
            fd = os.open(os.ctermid(), os.O_RDONLY)
            cr = ioctl_GWINSZ(fd)
            os.close(fd)
        except:
            pass
    if not cr:
        cr = (env.get('LINES', 25), env.get('COLUMNS', 80))

        ### Use get(key[, default]) instead of a try/catch
        #try:
        #    cr = (env['LINES'], env['COLUMNS'])
        #except:
        #    cr = (25, 80)
    return int(cr[1]), int(cr[0])

    
class ROSConsole():
    def __init__(self, arguments):
        self.buffer_ = []
        self.arguments = arguments
        # check arguments.node
        self.arguments.node = [n if n.startswith("/") else "/" + n
                               for n in self.arguments.node or []]
        self.arguments.exclude_node = [n if n.startswith("/") else "/" + n
                                       for n in self.arguments.exclude_node or []]
        self.lock_ = Lock()
        self.sub_ = rospy.Subscriber("/rosout", Log, self.rosoutCallback)
        self.timer_ = rospy.Timer(rospy.Duration(1 / 10.0), self.timerCallback)
    def rosoutCallback(self, msg):
        with self.lock_:
            self.buffer_.append(msg)
    def timerCallback(self, event):
        with self.lock_:
            table = [[levelString(msg), stampString(msg), "[%s]" % msg.name, coloredMessage(msg)] 
                     for msg in self.buffer_
                     if self.filterMessage(msg)]
            if len(table) == 0:
                return
            #print(tabulate(table, tablefmt="pipe"))
            self.prettyPrint(table)
            self.buffer_ = []
    def prettyPrint(self, table):
        header_length = max([len(" ".join(tab[:-1])) for tab in table])
        (width, height) = getTerminalSize()
        message_width = width - header_length - 1
        for tab in table:
            message = tab[-1]
            headers = " ".join(tab[:-1])
            if message_width <= 0:
                message_width = sys.maxint - 1        # give up prettye printing
            messages = [message[i*message_width:(i+1)*message_width]
                        for i in range(int(math.ceil(len(message) / float(message_width))))]
            messages_str = ("\n" + " " * (1 + header_length)).join(messages)
            padding = header_length - len(headers)
            print("%s %s%s" % (headers, " " * padding, messages_str) + Style.RESET_ALL)
    def filterMessage(self, msg):
        # node name
        show = True
        if self.arguments.node and len(self.arguments.node) > 0:
            if msg.name not in self.arguments.node:
                show = False
        if self.arguments.exclude_node and len(self.arguments.exclude_node) > 0:
            if msg.name in self.arguments.exclude_node:
                show = show and False
        # message level
        if self.arguments.level:
            show = show and levelGreaterEqualThan(msg, self.arguments.level)
        return show
if __name__ == "__main__":
    colorama.init()
    parser = argparse.ArgumentParser(description='Show rosout in your terminal')
    parser.add_argument('-n', '--node', help='Filter messages by node',
                        nargs='+')
    parser.add_argument('-N', '--exclude-node', help='Remove messages by node',
                        nargs='+')
    parser.add_argument('-l', '--level',
                        help='Filter messages by level (DEBUG, INFO, WARN, ERROR, FATAL)',
                        default = "DEBUG")
    rospy.init_node("ros_console", anonymous=True)
    args = parser.parse_args(rospy.myargv()[1:])
    console = ROSConsole(args)
    rospy.spin()

