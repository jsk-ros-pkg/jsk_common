#!/usr/bin/env python
import roslib; roslib.load_manifest('pddl_planner')
import rospy
import os

import actionlib
import pddl_msgs
from pddl_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('pddl_planner_client')
    client = actionlib.SimpleActionClient('pddl_planner',
                                          PDDLPlannerAction)
    client.wait_for_server()
    goal = PDDLPlannerGoal()
    goal.domain.name = "manip"
    goal.domain.requirements = ":typing"
    goal.domain.types = "object"
    goal.domain.predicates = ["(on ?obj0 ?obj1 - object)",
                                   "(clear ?obj - object)",
                                   "(ontable ?obj - object)",
                                   "(holding ?obj - object)",
                                   "(handempty)"]
    pickup = PDDLAction()
    pickup.name = "pickup"
    pickup.parameters = "(?obj - object)"
    pickup.precondition = "(and (ontable ?obj) (clear ?obj) (handempty))"
    pickup.effect = """(and (not (ontable ?obj)) (not (clear ?obj))
    (not (handempty)) (holding ?obj))"""
    putdown = PDDLAction()
    putdown.name = "putdown"
    putdown.parameters = "(?obj - object)"
    putdown.precondition = "(and (holding ?obj))"
    putdown.effect = """(and
    (not (holding ?obj))
    (ontable ?obj)
    (clear ?obj)
    (handempty))"""
    stack = PDDLAction()
    stack.name = "stack"
    stack.parameters = "(?obj0 ?obj1 - object)"
    stack.precondition = """(and 
    (holding ?obj0)
    (clear ?obj1))"""
    stack.effect = """(and
    (not (holding ?obj0))
    (not (clear ?obj1))
    (handempty)
    (on ?obj0 ?obj1)
    (clear ?obj0))"""
    unstack = PDDLAction()
    unstack.name = "unstack"
    unstack.parameters = "(?obj0 ?obj1 - object)"
    unstack.precondition = """(and 
                        (handempty)
                        (on ?obj0 ?obj1)
                        (clear ?obj0))"""
    unstack.effect = """(and
                 (not (handempty))
                 (not (on ?obj0 ?obj1))
                 (not (clear ?obj0))
                 (holding ?obj0)
                 (clear ?obj1))"""
    goal.domain.actions = [pickup, unstack, stack, putdown]
    goal.problem.name = "sample"
    goal.problem.domain = "manip"
    #goal.problem.objects = "a b c - obj"
    goal.problem.objects = [PDDLObject(name="a", type="obj"),
                            PDDLObject(name="b", type="obj"),
                            PDDLObject(name="c", type="obj")]
    goal.problem.initial = ["(on c a)", 
                            "(ontable a)",
                            "(ontable b)",
                            "(clear b)",
                            "(clear c)",
                            "(handempty)"]
    goal.problem.goal = "(and (on a b) (on b c))"
    print goal
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result() 
