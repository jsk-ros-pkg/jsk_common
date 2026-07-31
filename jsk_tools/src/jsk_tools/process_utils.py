import psutil


def search_pid_from_process_cmd(cmd):
    for p in psutil.process_iter():
        if cmd in p.cmdline():
            return p.pid
