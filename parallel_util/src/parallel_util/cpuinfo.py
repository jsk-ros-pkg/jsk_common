#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Ryohei Ueda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import paramiko
import os
from optparse import OptionParser

CPU_COUNT_COMMAND = 'python -c "import multiprocessing; print multiprocessing.cpu_count()"'
MEM_COUNT_COMMAND = 'python -c "import meminfo_total; print meminfo_total.meminfo_total()"'

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
        return_d[info[0]] = [info[1], info[2]]
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
        cpu_num_in = ssh_stdout.readline()
        cpu_num = int(cpu_num_in)
        try:                    # meminfo might be failed
            (ssh_stdin, ssh_stdout, ssh_stderr) = client.exec_command(MEM_COUNT_COMMAND)
            mem_num_in = ssh_stdout.readline()
            mem_num = int(mem_num_in)
        except Exception:
            mem_num = False
        return (host, cpu_num, mem_num)
    except Exception, e:
        if verbose:
            sys.stderr.write("[%s] connection missed\n" % (host))
        return (host, False)
