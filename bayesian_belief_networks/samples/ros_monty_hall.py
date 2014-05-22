#!/usr/bin/env python

# monty hall example
# https://github.com/eBay/bayesian-belief-networks/blob/master/docs/tutorial/tutorial.rst
# rosservice call /monty_hall/query []
# rosservice call /monty_hall/query [[guest_door,A]]
# rosservice call /monty_hall/query [[guest_door,A],[monty_door,B]]

import imp
try:
    imp.find_module('bayesian_belief_networks')
except:
    import roslib; roslib.load_manifest('bayesian_belief_networks')

from bayesian.examples.bbns.monty_hall import *
from bayesian_belief_networks.ros_utils import *

import rospy

rospy.init_node('monty_hall')
g = ros_build_bbn(
    f_prize_door,
    f_guest_door,
    f_monty_door,
    domains=dict(
        prize_door=['A', 'B', 'C'],
        guest_door=['A', 'B', 'C'],
        monty_door=['A', 'B', 'C']))

rospy.spin()



