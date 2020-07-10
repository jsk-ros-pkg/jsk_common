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
"""Managers multiple processors running on the ROS network completing a task.

The manager takes in a module that provides and processes the work items, it is oblivious of the ROS network. The module needs to provide the following functions:

.. code-block:: python

  # service side
  def service_start(args):

  # service side
  def service_processrequest(request):

  # server side
  def server_processresponse(response):

  # server side
  def server_requestwork():

  # server side
  def server_start(args):

  # server side
  def server_end():

  # launcher initialization
  def launcher_start(args):


There are 3 modes when launching the worker: initial launch setup, server manager, service runner.

"""
PKG='parallel_util'
import time
import inspect
import os, sys
import threading
import rospy, roslaunch
import roslaunch_caller
import cPickle as pickle
import re

from srv import PickledService, PickledServiceRequest, PickledServiceResponse
from cpuinfo import cpuinfos
from numpy import random
import std_msgs.msg

from xml.sax.saxutils import quoteattr

class EvaluationServerThread(threading.Thread):
    def __init__(self, service, finishcb):
        threading.Thread.__init__(self)
        self.service = service
        self.finishcb = finishcb
        self.ok = True
        self.starteval = threading.Condition(threading.Lock())
        self.req = None
        self.servicechecked = False
        self.num = -1
        self.starttime = 0

    def run(self):
        with self.starteval:
            while self.ok:
                self.starteval.wait()
                if not self.ok:
                    break
                if self.req is None:
                    rospy.logwarn('dummy command?')
                    continue
                self.starttime = time.time()
                res = self.service(self.req)
                if res is not None and self.ok:
                    self.finishcb(pickle.loads(res.output))
                    rospy.logdebug('%s: job %d done'%(self.service.resolved_name,self.num))
                self.req = None

class EvaluationServer(object):
    def __init__(self,module,servicenames,numbatchjobs=1):
        self.evallock = threading.Lock()
        self.allthreads = []
        self.module = module
        self.numbatchjobs=numbatchjobs
        assert(numbatchjobs>0)
        self.setservices(servicenames)
        self.pubResults = rospy.Publisher('results', std_msgs.msg.String)
        
    def __del__(self):
        self.shutdownservices()

    def shutdownservices(self):
        if len(self.allthreads) > 0:
            rospy.loginfo('shutting down services')
            for t in self.allthreads:
                t.ok = False
                with t.starteval:
                    t.starteval.notifyAll()
            rospy.loginfo('services have shutdown')
            self.allthreads = []

    def setservices(self,servicenames):
        self.shutdownservices()
        rospy.loginfo('starting service threads...')
        self.allthreads = [EvaluationServerThread(rospy.ServiceProxy(name, PickledService,persistent=True), self.processResult) for name in servicenames]
        for t in self.allthreads:
            t.start()

    def processResult(self,responses):
        if responses is not None:
            with self.evallock:
                for response in responses:
                    self.module.server_processresponse(*response)
                    if self.pubResults.get_num_connections() > 0:
                        #rospy.loginfo('republish the results')
                        self.pubResults.publish(pickle.dumps(response))

    def QueryServiceInfo(self,req):
        threadinfo = []
        for t in self.allthreads:
            threadinfo.append([t.service.resolved_name, t.num, time.time()-t.starttime, t.req])
        return PickledServiceResponse(output=pickle.dumps(threadinfo))

    def run(self):
        self.queryservice = rospy.Service('queryserviceinfo', PickledService, self.QueryServiceInfo)

        starttime = time.time()
        busythreads = self.allthreads[:]
        # reset the evaluated threads
        for t in self.allthreads:
            t.starteval.acquire()
            t.starteval.release()

        num = 0
        requests = []
        doshutdown = False
        while not doshutdown:
            request = self.module.server_requestwork()
            if request is not None:
                requests.append(request)
                num += 1
            if request is None or len(requests) >= self.numbatchjobs:
                service = None
                while service == None and not doshutdown:
                    for t in busythreads:
                        if t.req is None and t.servicechecked:
                            service = t
                            break
                    if service == None:
                        random.shuffle(busythreads)
                        for t in busythreads:
                            if t.req is None and not t.servicechecked:
                                try:
                                    t.service.wait_for_service(0.5)
                                    rospy.loginfo('service %s is verified'%t.service.resolved_name)
                                    t.servicechecked = True
                                    service = t
                                except rospy.ROSInterruptException as e:
                                    rospy.loginfo('timed out (%s)'%str(e))
                                    doshutdown = True
                                    break
                                except rospy.ROSException as e:
                                    rospy.loginfo('service %s timed out (%s)'%(t.service.resolved_name,str(e)))
                                break

                    if service is None:
                        time.sleep(0.01)
                if service is not None:
                    with service.starteval:
                        rospy.logdebug('%s: job %d'%(t.service.resolved_name,num))
                        service.num = num
                        service.req = PickledServiceRequest(input = pickle.dumps(requests))
                        requests = []
                        service.starteval.notifyAll()
            if request is None:
                break

        # wait for all threads to finish
        rospy.loginfo('waiting for all threads to finish')
        for t in busythreads:
            repeat = True
            if t.req is not None:
                rospy.loginfo('shutting down service %s'%t.service.resolved_name)
            try:
                while(repeat):
                    with t.starteval:
                        if t.req is None:
                            repeat = False
            except KeyboardInterrupt as e:
                rospy.loginfo('ignore service %s'%t.service.resolved_name)

        rospy.loginfo('services finished processing, total time: %f'%(time.time()-starttime))


def LaunchNodes(module,serviceaddrs=[('localhost','')],rosnamespace=None,args='',numbatchjobs=1,log_level='info',programname=None,feedbackfn=None, rosuser=None):
    """feedbackfn is called once in a while inside the loop. If it returns True, then will shutdown the roslaunch process
    """
    assert(len(serviceaddrs)>0)
    starttime = time.time()
    servicenames = ''
    if programname is None:
        programname = os.path.split(sys.argv[0])[1]
    modulepath=os.path.split(os.path.abspath(inspect.getfile(module)))[0]
    #processedargs = quoteattr(args + ' --loglevel=%s '%log_level)
    userattr = ''
    if rosuser is not None:
        userattr = ' user="%s" '%rosuser
    nodes = """<machine timeout="30" name="localhost" address="localhost" %s default="true"/>\n"""%userattr
    for i,serviceaddr in enumerate(serviceaddrs):
        serviceargs = """--startservice --module=%s --log_level=%s --args='%s --log_level=%s'"""%(module.__name__,log_level,args,log_level)
        nodes += """<machine timeout="30" name="m%d" address="%s" %s default="false" %s/>\n"""%(i,serviceaddr[0],serviceaddr[1],userattr)
        nodes += """<node machine="m%d" name="openraveservice%d" pkg="%s" type="%s" args=%s output="log" cwd="node" respawn="true">\n  <remap from="openraveservice" to="openraveservice%d"/>\n</node>"""%(i,i,PKG,programname,quoteattr(serviceargs),i)
        servicenames += ' --service=openraveservice%d '%i

    processedargs = """--numbatchjobs=%d --log_level=%s --module=%s %s --args='%s --log_level=%s'"""%(numbatchjobs,log_level,module.__name__,servicenames,args,log_level)
    nodes += """<node machine="localhost" name="openraveserver" pkg="%s" type="%s" args=%s output="screen" cwd="node" respawn="false"/>\n"""%(PKG,programname,quoteattr(processedargs))
    xml_text = '<?xml version="1.0" encoding="utf-8"?>\n<launch>\n<env name="PYTHONPATH" value="$(optenv PYTHONPATH):%s"/>\n'%modulepath
    if 'LANG' in os.environ:
        xml_text += '<env name="LANG" value="%s"/>\n'%os.environ['LANG']
    if rosnamespace is not None and len(rosnamespace) > 0:
        xml_text += """<group ns="%s">\n%s</group>"""%(rosnamespace,nodes)
    else:
        xml_text += nodes
    xml_text += '\n</launch>\n'
    print(xml_text)
    roslaunch.pmon._shutting_down = False # roslaunch registers its own signal handlers and shuts down automatically on sigints
    launchscript = roslaunch_caller.ScriptRoslaunch(xml_text)
    try:
        launchscript.start()
        controlname = [name for name in launchscript.pm.get_active_names() if name.find('openraveserver')>=0]
        while True:
            controlproc = launchscript.pm.get_process(controlname[0])
            if controlproc is None or not controlproc.is_alive():
                break
            if feedbackfn is not None:
                if feedbackfn():
                    rospy.loginfo('terminate by feedback function')
                    break
            else:
                time.sleep(1)
        rospy.loginfo('roslaunch %s finished in %ss'%(module.__name__,time.time()-starttime))
    finally:
        rospy.loginfo('shutting down')
        launchscript.shutdown()

def StartService(module,args):
    module.service_start(args)
    def service_call(req):
        responses = [module.service_processrequest(*request) for request in pickle.loads(req.input)]
        return PickledServiceResponse(output=pickle.dumps(responses))
    
    s = rospy.Service('openraveservice', PickledService, service_call)
    return s
