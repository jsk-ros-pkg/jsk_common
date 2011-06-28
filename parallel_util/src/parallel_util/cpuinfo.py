import sys
import paramiko
import os
from optparse import OptionParser

CPU_COUNT_COMMAND = 'python -c "import multiprocessing; print multiprocessing.cpu_count()"'

def cpuinfos(hosts=[], from_cssh_file = None,
             cssh_group = None,
             timeout = None, port = 22, verbose = False):
    if from_cssh_file and cssh_group:
        hosts = parse_cssh_config(from_cssh_file, cssh_group)
    cpuinfos = [collect_cpuinfo(host, verbose, timeout) for host in hosts]
    valid_cpuinfos = [info for info in cpuinfos if info[1] != False]
    # convert valid_cpuinfos to dict
    return_d = {}
    for info in valid_cpuinfos:
        return_d[info[0]] = [info[1]]
    return return_d

def parse_cssh_config(config_file, group):
    infile = open(config_file, "r")
    while True:
        org_line = infile.readline()
        line = org_line.strip() # remove white chars
        if org_line == "":
            raise Exception("cannot find group %s at %s" % (group,
                                                            config_file))
        elif line.startswith(group):
            # matched!
            return line.split()[1:]

def collect_cpuinfo(host, verbose, timeout):
    try:
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        if verbose:
            sys.stderr.write("[%s] connecting\n" % (host))
        client.connect(host, timeout=timeout)
        if verbose:
            sys.stderr.write("[%s] connection established\n" % (host))
        (ssh_stdin, ssh_stdout, ssh_stderr) = client.exec_command(CPU_COUNT_COMMAND)
        cpu_num = int(ssh_stdout.readline())
        return (host, cpu_num)
    except Exception:
        if verbose:
            sys.stderr.write("[%s] connection missed\n" % (host))
        return (host, False)
