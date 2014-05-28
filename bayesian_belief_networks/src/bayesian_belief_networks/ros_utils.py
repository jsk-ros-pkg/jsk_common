#!/usr/bin/env python

from bayesian_belief_networks.srv import Query, QueryResponse
from bayesian_belief_networks.msg import Result, Observation
from bayesian.bbn import *

import rospy
_bnn_object = None

def service_query(req):
    global _bnn_object

    kwds = dict()
    rospy.loginfo('query = %s'%req.query)
    for observation in req.query:
        kwds[observation.node] = observation.evidence
    result = _bnn_object.query(**kwds)
    rospy.loginfo('results = %s'%result)
    #
    res = QueryResponse()
    for (node, value), prob in result.items():
        r = Result(node,value,prob)
        res.results.append(r)
    res.results = sorted(res.results, key=lambda x: x.node)
    return res


def ros_build_bbn(*args, **kwds):
    global _bnn_object
    "call rospy.init_node() before build_ros_bbn() and call rospy.spin() after that"
    _bnn_object = build_bbn(*args, **kwds)
    rospy.Service('~query', Query, service_query)
    return _bnn_object

