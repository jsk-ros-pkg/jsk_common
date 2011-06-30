#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\brief Kevin Watts and Ken Conley

#import roslib
#roslib.load_manifest('parallel_util')

import roslaunch
import roslaunch.core
import roslaunch.parent
import roslaunch.pmon
import roslaunch.config

import rospy

## ScriptRoslaunch is a specialization of the roslaunch parent for
## using the roslaunch API. It allows registration of a process listener with
## the launched parent and also overrides the default roslaunch config
## initialiation in order to load from strings instead of files.
class ScriptRoslaunch(roslaunch.parent.ROSLaunchParent):

    ## @param self
    ## @param launch_str str: roslaunch XML config (as a string, not a
    ## file path)
    ## @param process_listener (optional): process listener to
    ## register with roslaunch
    def __init__(self, launch_str, process_listener=None):
        self.launch_strs = [launch_str]
        # use run_id from parent
        run_id = rospy.get_param('/run_id')
        if process_listener:
            process_listeners = [process_listener]
        else:
            process_listeners = []
        super(ScriptRoslaunch, self).__init__(run_id, [], process_listeners=process_listeners)
        
    def _load_config(self):
        self.config = roslaunch.config.load_config_default([], self.port,
                                          roslaunch_strs=self.launch_strs)
    
def launch_core():
    # Launch the core, generate the run_id

    # in the future this may need a rewrite as roslaunch gets the
    # ability to remotely launch cores, but for now this is fine
    config = roslaunch.ROSLaunchConfig()
    config.master.auto = config.master.AUTO_START
        
    run_id = roslaunch.core.generate_run_id()
    core_launcher = roslaunch.ROSLaunchRunner(run_id, config)
    core_launcher.launch()
    
    return core_launcher

