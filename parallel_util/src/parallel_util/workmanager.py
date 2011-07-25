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

from srv import PickledService, PickledServiceRequest, PickledServiceResponse
from cpuinfo import cpuinfos

class EvaluationServerThread(threading.Thread):
    def __init__(self, service, finishcb):
        threading.Thread.__init__(self)
        self.service = service
        self.finishcb = finishcb
        self.ok = True
        self.starteval = threading.Condition(threading.Lock())
        self.req = None
        self.servicechecked = False

    def run(self):
        with self.starteval:
            while self.ok:
                self.starteval.wait()
                if not self.ok:
                    break
                if self.req is None:
                    rospy.logwarn('dummy command?')
                    continue
                res = self.service(self.req)
                if res is not None and self.ok:
                    self.finishcb(pickle.loads(res.output))
                self.req = None

class EvaluationServer(object):
    def __init__(self,module,servicenames,numbatchjobs=1):
        self.evallock = threading.Lock()
        self.allthreads = []
        self.module = module
        self.numbatchjobs=numbatchjobs
        assert(numbatchjobs>0)
        self.setservices(servicenames)

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
            
    def run(self):
        starttime = time.time()
        busythreads = self.allthreads[:]
        # reset the evaluated threads
        for t in self.allthreads:
            t.starteval.acquire()
            t.starteval.release()

        num = 0
        requests = []
        while True:
            request = self.module.server_requestwork()
            if request is not None:
                requests.append(request)
                num += 1
            if request is None or len(requests) >= self.numbatchjobs:
                rospy.logdebug('job %d'%num)
                service = None
                while service == None:
                    for t in busythreads:
                        if t.req is None:
                            if not t.servicechecked:
                                try:
                                    t.service.wait_for_service(2.0)
                                    rospy.loginfo('service %s is verified'%t.service.resolved_name)
                                    t.servicechecked = True
                                except rospy.ROSException,e:
                                    rospy.loginfo('service %s timed out'%t.service.resolved_name)
                                    continue
                            service = t
                            break
                    if service is None:
                        time.sleep(0.01)
                with service.starteval:
                    service.req = PickledServiceRequest(input = pickle.dumps(requests))
                    requests = []
                    service.starteval.notifyAll()
            if request is None:
                break

        # wait for all threads to finish
        rospy.loginfo('waiting for all threads to finish')
        for t in busythreads:
            repeat = True
            while(repeat):
                with t.starteval:
                    if t.req is None:
                        repeat = False

        rospy.loginfo('finished, total time: %f'%(time.time()-starttime))


def LaunchNodes(module,serviceaddrs=[('localhost','')],rosnamespace=None,args='',numbatchjobs=1):
    servicenames = ''
    programname = os.path.split(sys.argv[0])[1]
    modulepath=os.path.split(os.path.abspath(inspect.getfile(module)))[0]
    nodes = """<machine name="localhost" address="localhost" default="true"/>\n"""
    for i,serviceaddr in enumerate(serviceaddrs):
        nodes += """<machine name="m%d" address="%s" default="false" %s/>\n"""%(i,serviceaddr[0],serviceaddr[1])
        nodes += """<node machine="m%d" name="openraveservice%d" pkg="%s" type="%s" args="--startservice --module=%s --args='%s'" output="log" cwd="node">\n  <remap from="openraveservice" to="openraveservice%d"/>\n</node>"""%(i,i,PKG,programname,module.__name__,args,i)
        servicenames += ' --service=openraveservice%d '%i
    nodes += """<node machine="localhost" name="openraveserver" pkg="%s" type="%s" args=" --numbatchjobs=%d --module=%s %s --args='%s'" output="screen" cwd="node"/>\n"""%(PKG,programname,numbatchjobs,module.__name__,servicenames,args)
    xml_text = '<launch>\n<env name="PYTHONPATH" value="$(optenv PYTHONPATH):%s"/>\n'%modulepath
    if rosnamespace is not None and len(rosnamespace) > 0:
        xml_text += """<group ns="%s">\n%s</group>"""%(rosnamespace,nodes)
    else:
        xml_text += nodes
    xml_text += '\n</launch>\n'
    print xml_text
    roslaunch.pmon._shutting_down = False # roslaunch registers its own signal handlers and shuts down automatically on sigints
    launchscript = roslaunch_caller.ScriptRoslaunch(xml_text)
    launchscript.start()
    try:
        starttime = time.time()
        controlname = [name for name in launchscript.pm.get_active_names() if name.find('openraveserver')>=0]
        while True:
            controlproc = launchscript.pm.get_process(controlname[0])
            if controlproc is None or not controlproc.is_alive():
                break
            time.sleep(1)
        rospy.loginfo('%s finished in %ss'%(module.__name__,time.time()-starttime))
    finally:
        rospy.loginfo('shutting down')
        launchscript.shutdown()

def StartService(module,args):
    module.service_start(args.split())
    def service_call(req):
        responses = [module.service_processrequest(*request) for request in pickle.loads(req.input)]
        return PickledServiceResponse(output=pickle.dumps(responses))
    
    s = rospy.Service('openraveservice', PickledService, service_call)
    return s
