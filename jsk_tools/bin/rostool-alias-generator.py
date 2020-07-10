#!/usr/bin/env python
# -*- coding: utf-8 -*-

# only supports bash as a shell

import os
import sys
import re
import commands
import yaml
# global variables

def ros_master_uri(m):
    return "http://%s:%s"%(m['name'], m['port'])

def color_string(n, str):
    return "\\\[\033[01;%dm\\]%s\\[\\033[00m\\]"%(n, str)

def gen_ps1_value(m):
    return "'[%s]%s:%s\\$ '"%(color_string(31, m['robot'] + "/" + m['name']),
                              color_string(32, "\\u@\\h"), 
                              color_string(34, "\\w"))

def gen_ros_alias_name(prog_name, m):
    return prog_name + "_" + m['name']

def gen_bash_alias_name(m):
    return "bash_" + m['name']

def misc_unix_env(m):
    if m.has_key('env'):
        envs = m['env']
        all_keys = envs.keys()
        ret = ""
        for i in all_keys:
            ret = ret + "%s=%s " % (i, envs[i])
            return ret
    else:
        return ""

def gen_bash_alias_command(m):
    return 'alias %s="ROS_MASTER_URI=%s ROBOT=%s %s PS1=%s bash --norc"' % (gen_bash_alias_name(m),
                                                                            ros_master_uri(m), 
                                                                            m['robot'],
                                                                            misc_unix_env(m),
                                                                            gen_ps1_value(m))

def gen_ros_alias_command(m, i):
    return 'function %s() { %s -c "%s $*" ; }' % (gen_ros_alias_name(i, m),
                                                  gen_bash_alias_name(m), i)

if len(sys.argv) < 2:
    print('You need to specify yaml file')
    exit(1)

robot_machines = yaml.load(open(sys.argv[1]).read())
ros_bin_directory = os.environ['ROS_ROOT'] + "/bin"
ros_bin_programs = [i for i in os.listdir(ros_bin_directory) 
                    if i.startswith("r")]
complete_strings = commands.getoutput("bash -i -c \"complete\"").split("\n")

# only supports bash
def gen_ros_completion(ros_program, aliased_command):
    _complete_strings = [x for x in complete_strings
                         if x.endswith(ros_program)]
    if not len(_complete_strings) == 0:
        complete_string = _complete_strings[0]
        ros_complete_strings = re.sub(ros_program,
                                      aliased_command,
                                      complete_string)
        return ros_complete_strings
    else:
        return False

# generate bash command
for m in robot_machines:
    print(gen_bash_alias_command(m['machine']))

# generate roslaunch_hrp2007v
for p in ros_bin_programs:
    for m in robot_machines:
        aliased_command_definition = gen_ros_alias_command(m['machine'], p)
        aliased_command = gen_ros_alias_name(p, m['machine'])
        complete = gen_ros_completion(p, aliased_command)
        print(aliased_command_definition)
        if complete:
            print(complete)
            
