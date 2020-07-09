#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
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
import roslib; roslib.load_manifest('parallel_util')
from parallel_util import workmanager
import sys,os
import rospy
from rosgraph import masterapi
from optparse import OptionParser
from shlex import split

if __name__=='__main__':
    parser = OptionParser(description='There are 3 modes when launching the worker: initial launch setup, server manager, service runner',
                          usage='%prog [options]')
    parser.add_option('--module', action='store', type='string', dest='modulename',default=None,
                      help='module to use to execute the client/server')
    parser.add_option('--args', action='store', type='string', dest='args',default='',
                      help='arguments to pass in to each function')
    parser.add_option('--startservice', action='store_true', dest='startservice',default=False,
                      help='If set, will start a service on the ROS network offering to evaluate kinematics equations')
    parser.add_option('--service', action='append', type='string', dest='servicenames',default=[],
                      help='The services used to evaluate kinematics')
    parser.add_option('--numbatchjobs', action='store', type='int', dest='numbatchjobs',default=1,
                      help='The number of batch jobs to request and send to ROS at one time (used to reduce bandwidth)')
    parser.add_option('--launchservice', action='append', dest='launchservices',default=[],
                      help="""If specified, will roslaunch the services and setup the correct bindings for parallel processing (recommended). Usage: "python prog.py --launchservice='4*localhost' ..." """)
    parser.add_option('--csshgroup', action='store', type='string', dest='csshgroup',default=None,
                      help='The group of computers to specify when launching')
    parser.add_option('--log_level', action='store', type='string', dest='log_level',default='info',
                      help='The ros loglevel to set rospy at: debug, info, warn, error, fatal')
    #print('python path: ',os.environ['PYTHONPATH'])
    (options, args) = parser.parse_args()
    log_level = rospy.INFO
    if options.log_level == 'debug':
        log_level = rospy.DEBUG
    elif options.log_level == 'info':
        log_level = rospy.INFO
    elif options.log_level == 'warn':
        log_level = rospy.WARN
    elif options.log_level == 'error':
        log_level = rospy.ERROR
    elif options.log_level == 'fatal':
        log_level = rospy.FATAL

    module = __import__(options.modulename, globals(), locals(), options.modulename.rsplit('.',1)[1:])
    
    if options.startservice:
        rospy.init_node('servicenode',anonymous=True,log_level=log_level)
        s=workmanager.StartService(module,split(options.args))
        rospy.spin()
    elif len(options.servicenames) > 0:
        module.server_start(split(options.args))
        rospy.init_node('servicenode',anonymous=True,log_level=log_level)
        try:
            self = workmanager.EvaluationServer(module,options.servicenames,numbatchjobs=options.numbatchjobs)
            self.run()
            self.shutdownservices()
        finally:
            module.server_end()
    else:
        assert(masterapi.is_online())
        print('split args: ',split(options.args))
        options.args += ' ' + module.launcher_start(split(options.args))
        serviceaddrs = []
        if options.csshgroup is not None:
            infos = cpuinfos(from_cssh_file = os.path.join(os.environ["HOME"], ".cssh-clusters"), cssh_group = options.csshgroup, verbose = False, timeout = None)
            for addr, cpuinfo in infos.items():
                for i in range(cpuinfo[0]):
                    serviceaddrs.append([addr,''])
        for launchservice in options.launchservices:
            launchservice = launchservice.strip()
            pos = launchservice.find(' ')
            if pos >= 0:
                addr = launchservice[0:pos]
                args = launchservice[pos+1:]
            else:
                addr = launchservice
                args = ''
            posnum = addr.find('*')
            if posnum >= 0:
                numprocesses=int(addr[0:posnum])
                addr = addr[posnum+1:].strip()
                for i in range(numprocesses):
                    serviceaddrs.append([addr,args])
            else:
                serviceaddrs.append([addr,args])
        workmanager.LaunchNodes(module,serviceaddrs=serviceaddrs,rosnamespace=options.modulename,args=options.args,numbatchjobs=options.numbatchjobs,log_level=options.log_level,programname=os.path.split(sys.argv[0])[1])
    sys.exit(0)
