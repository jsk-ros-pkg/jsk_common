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
PKG='parallel_util'
import roslib; roslib.load_manifest(PKG)
from parallel_util import workmanager
import sys,os
import rospy
from optparse import OptionParser
import cPickle as pickle

from parallel_util.srv import PickledService, PickledServiceRequest, PickledServiceResponse

if __name__=='__main__':
    queryservice = rospy.ServiceProxy(sys.argv[1], PickledService,persistent=False)
    res=queryservice()
    threadinfo = pickle.loads(res.output)
    for name, num, duration, req in threadinfo:
        print('%s: num=%d, duration=%f, requests:'%(name,num,duration))
        if req is not None:
            print(pickle.loads(req.input))
        print('--------')
