#!/usr/bin/env python

from subprocess import Popen
import sys
import signal
import os
from time import sleep
sleep_time = int(sys.argv[1])
commands = sys.argv[2:]
print(commands)

p = Popen(commands)
sleep(sleep_time)
os.kill(p.pid, signal.SIGINT)
