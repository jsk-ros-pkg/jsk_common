#!/usr/bin/python

import roslib; roslib.load_manifest('pddl_planner_viewer')
import rospy
import pddl_msgs
from pddl_msgs.msg import *
import actionlib

import wx

class PDDLPlannerViewer(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title, size=(400, 500))

        # make a root panel and sizer
        self._panel = wx.Panel(self, -1)
        self._sizer = wx.GridBagSizer(4, 4)
        # static text
        state_text = wx.StaticText(self._panel, -1,
                                   'Initial States which are True')
        plan_text = wx.StaticText(self._panel, -1, 'Plan')
        self._problem_text = wx.StaticText(self._panel, -1, 'problem name: ')
        self._domain_text = wx.StaticText(self._panel, -1, 'domain name: ')
        self._sizer.Add(self._problem_text, (0, 0),
                        flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)
        self._sizer.Add(self._domain_text, (1, 0),
                        flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)
        self._sizer.Add(state_text, (2, 0),
                        flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)
        self._sizer.Add(plan_text, (8, 0),
                        flag=wx.TOP | wx.LEFT | wx.BOTTOM, border=5)
        
        self._state_list = wx.ListBox(self._panel, -1, style=wx.LB_ALWAYS_SB)
        self._sizer.Add(self._state_list, (3, 0), (5, 3),
                        wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        
        
        self._plan_list = wx.ListBox(self._panel, -1, style=wx.LB_ALWAYS_SB)
        self._sizer.Add(self._plan_list, (9, 0), (3, 3),
                        wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        
        self._sizer.AddGrowableCol(0)
        self._sizer.AddGrowableRow(3)
        self._sizer.AddGrowableRow(9)
        self._sizer.SetEmptyCellSize((5, 5))
        self._panel.SetSizer(self._sizer)
        self._need_to_refresh_goal = False
        self._need_to_refresh_plan_results = False
        
        self.Bind(wx.EVT_IDLE, self.update_plan)
        rospy.Subscriber("pddl_planner/result", PDDLPlannerActionResult,
                         self.result_listen)
        rospy.Subscriber("pddl_planner/goal", PDDLPlannerActionGoal,
                         self.goal_listen)
        self.Centre()
        self.Show(True)
        
    def goal_listen(self, data):
        self._goal = data.goal
        self._need_to_refresh_goal = True
        
    # result listener
    def result_listen(self, data):
        self._plan_results = data.result.sequence
        self._need_to_refresh_plan_results = True

        
    def update_plan(self, dummy):
        if self._need_to_refresh_plan_results:
            self._plan_list.Clear()
            i = 0
            for r in self._plan_results:
                self._plan_list.Append(str(i) + " " + 
                                       str(tuple([r.action]) + tuple(r.args)))
                i = i + 1
            self._need_to_refresh_plan_results = False
            
        if self._need_to_refresh_goal:
            self._state_list.Clear()
            for i in self._goal.problem.initial:
                self._state_list.Append(str(i))
            self._need_to_refresh_goal = False
            self._problem_text.SetLabel('problem name: '
                                        + self._goal.problem.name)
            self._domain_text.SetLabel('domain name: '
                                        + self._goal.domain.name)


# main
if __name__ == '__main__':
    rospy.init_node('pddl_planner_viewer')
    app = wx.App()
    PDDLPlannerViewer(None, -1, 'PDDL Plan')
    app.MainLoop()
