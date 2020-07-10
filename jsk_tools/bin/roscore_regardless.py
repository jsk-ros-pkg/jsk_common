#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import rospy
import signal
import subprocess
import sys
import time
from jsk_topic_tools.master_util import isMasterAlive

# Global variables
g_process_object = None
g_is_posix = 'posix' in sys.builtin_module_names


def printLog(fmt, **args):
    print("\x1b[32m" + fmt % args + '\x1b[39m')


def runProcess(cmds):
    """Spawn child process with given commands"""
    global g_process_object
    g_process_object = subprocess.Popen(
        args=cmds,
        close_fds=g_is_posix,
        env=os.environ.copy(),
        preexec_fn=os.setpgrp(),
    )


def killDescendentProcesses(ppid):
    """Kill descendent processes of given pid"""
    try:
        output = subprocess.check_output(['ps', '--ppid=' + str(ppid), '--no-headers'])
    except subprocess.CalledProcessError:
        # ppid does not exist any more
        return True

    for process_line in output.split('\n'):
        strip_process_line = process_line.strip()
        if strip_process_line:
            pid = int(strip_process_line.split(' ')[0])
            name = strip_process_line.split(' ')[-1]
            printLog('Killing %s [%d]' % (name, pid))
            os.kill(pid, signal.SIGINT)


def killChildProcess(p, sigint_timeout, sigterm_timeout, signum=None):
    # Return exit code if the process is already exited.
    if p is None:
        return 0
    if p.poll() is not None:
        return p.poll()

    # If it's still running, send signals to kill.
    try:
        # 1. SIGINT
        p.send_signal(signal.SIGINT)
        timeout = time.time() + sigint_timeout
        while time.time() < timeout:
            if p.poll() is not None:
                return p.poll()
            time.sleep(0.1)

        # 2. SIGTERM
        printLog("Escalated to SIGTERM")
        p.send_signal(signal.SIGTERM)
        timeout = time.time() + sigterm_timeout
        while time.time() < timeout:
            if p.poll() is not None:
                return p.poll()
            time.sleep(0.1)

        # 3. SIGKILL
        printLog("Escalated to SIGKILL")
        p.kill()
        p.wait()

        return p.poll()

    except Exception as e:
        printLog("Failed to kill child process: %s" % str(e))

    # Here should never be reached.
    return 0


def killProcess(sigint_timeout, sigterm_timeout):
    """Kill the running child process by sending signals.
       First send SIGINT as normal exit, and then is escalated to SIGKILL if the process is still alive.
       Return value is the exit code of the process.
    """
    global g_process_object

    p = g_process_object
    g_process_object = None

    exit_code = killChildProcess(p, sigint_timeout, sigterm_timeout)

    if p is not None:
        try:
            killDescendentProcesses(p.pid)
        except Exception as e:
            printLog("Failed to kill descendent processes: %s" % str(e))

    return exit_code


def signalHandler(signum, stack):
    global g_process_object
    if g_process_object and g_process_object.poll() is None:
        g_process_object.send_signal(signum)
        printLog("Sent signal %d to running process" % signum)


def parse_args(args):
    p = argparse.ArgumentParser()
    p.add_argument("commands", nargs=argparse.REMAINDER)
    p.add_argument("--respawn", "-r", action="store_true",
                   help="respawn if child process stops")
    p.add_argument("--timeout", type=int, default=10,
                   help="Timeout to verify if rosmaster is alive by ping command in seconds")
    p.add_argument("--ping-trials", type=int, default=1,
                   help="If ping fails PING_TRIALS times, master is regarded as dead")
    p.add_argument("--sigint-timeout", type=int, default=20,
                   help="Timeout to escalete from sigint to sigterm to kill child processes")
    p.add_argument("--sigterm-timeout", type=int, default=10,
                   help="Timeout to escalete from sigterm to sigkill to kill child processes")
    args = p.parse_args()
    return (args.commands, args.respawn, args.timeout, args.sigint_timeout, args.sigterm_timeout,
            args.ping_trials)


def main(args):
    cmds, respawn, timeout, sigint_timeout, sigterm_timeout, trials = parse_args(args)
    exit_code = 0
    previous_master_state = None
    try:
        while True:
            master_state = isMasterAlive(timeout_sec=timeout, trials=trials)
            if g_process_object and g_process_object.poll() is not None:
                # Child process exited
                pid = g_process_object.pid
                exit_code = g_process_object.poll()
                printLog("Child process exited [%d] (code: %d)" % (pid, exit_code))
                if respawn:
                    printLog("Restarting process")
                    runProcess(cmds)
                    printLog("Process restarted [%d]" % g_process_object.pid)
                else:
                    break
            if not master_state and previous_master_state:
                # Master is gone dead
                pid = g_process_object.pid
                printLog("Killing running process")
                exit_code = killProcess(sigint_timeout, sigterm_timeout)
                printLog("Killed running process [%d] (code: %d)" % (pid, exit_code))
            elif master_state and not previous_master_state:
                # Master is now alive
                printLog("Starting process")
                runProcess(cmds)
                printLog("Process started [%d]" % g_process_object.pid)
            previous_master_state = master_state
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        printLog("Cleaning up processes")
        exit_code = killProcess(sigint_timeout, sigterm_timeout)

    return exit_code


if __name__ == '__main__':
    exit(main(rospy.myargv()[1:]))
