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
ARCH_CHECK_COMMAND = 'python -c "import platform; print platform.machine()"'
ROS_CHECK_COMMAND = """sh -c 'rospack list >/dev/null 2>&1 && python -c "import roslib" && echo True || echo False'"""
ROSPORT_COMMAND = """python -c "import roslib; roslib.load_manifest('rosgraph'); import rosgraph.masterapi; print not(rosgraph.masterapi.is_online('http://localhost:%s'))" """
def cpuinfos(hosts=[], from_cssh_file = None,
             cssh_group = None,
             timeout = None, ros_port = 11311, verbose = False):
    if from_cssh_file and cssh_group:
        hosts = parse_cssh_config(from_cssh_file, cssh_group)
    cpuinfos = [collect_cpuinfo(host, ros_port, verbose, timeout) for host in hosts]
    valid_cpuinfos = [info for info in cpuinfos if info[1] != False]
    # convert valid_cpuinfos to dict
    return_d = {}
    for info in valid_cpuinfos:
        return_d[info[0]] = [info[1], info[2], info[3]]
    port_available_p = all([info[4] == True for info in valid_cpuinfos])
    return (return_d, port_available_p)

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

def collect_cpuinfo(host, ros_port, verbose, timeout):
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
        try:                    # meminfo might be failed
            (ssh_stdin, ssh_stdout, ssh_stderr) = client.exec_command(ARCH_CHECK_COMMAND)
            arch = ssh_stdout.readline().strip()
        except Exception:
            arch = False
        (ssh_stdin, ssh_stdout, ssh_stderr) = client.exec_command(ROS_CHECK_COMMAND)
        ros_p = ssh_stdout.readline().strip()
        if ros_p == "True":
            (ssh_stdin, ssh_stdout, ssh_stderr) = client.exec_command(ROSPORT_COMMAND % (ros_port))
            port_available_p = ssh_stdout.readline().strip() == "True"
            return (host, cpu_num, mem_num, arch, port_available_p)
        else:
            raise "ros is not installed"
    except Exception, e:
        if verbose:
            sys.stderr.write("[%s] connection missed or ROS is not installed\n" % (host))
        return (host, False)
