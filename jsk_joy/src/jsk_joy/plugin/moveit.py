import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from std_msgs.msg import String, Empty
import xml.etree.ElementTree as ET

class JoyMoveIt(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyMoveIt')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/map')
    # parse srdf to get planning_groups
    srdf = rospy.get_param("/robot_description_semantic")
    root = ET.fromstring(srdf)
    self.planning_groups = rospy.get_param("~planning_groups", [])
    if (self.planning_groups) == 0: # auto detect
      for group in root.iter("group"):
        counter = 0
        for l in group.iter("chain"):
          counter = counter + 1
        if counter > 0:
          self.planning_groups.append(group.attrib["name"])
    self.current_planning_group_index = 0
    self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String)
    self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty)
    self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty)
    self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty)
    self.counter = 0
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.triangle and not latest.triangle:
      self.current_planning_group_index = self.current_planning_group_index + 1
      if self.current_planning_group_index >= len(self.planning_groups):
        self.current_planning_group_index = 0
      self.plan_group_pub.publish(self.planning_groups[self.current_planning_group_index])
    elif status.cross and not latest.cross:
      self.current_planning_group_index = self.current_planning_group_index - 1
      if self.current_planning_group_index < 0:
        self.current_planning_group_index = len(self.planning_groups) - 1
      self.plan_group_pub.publish(self.planning_groups[self.current_planning_group_index])
    elif status.square and not latest.square:
      self.plan_pub.publish(Empty())
    elif status.circle and not latest.circle:
      self.execute_pub.publish(Empty())
    self.counter = self.counter + 1
    if self.counter % 10:
      self.update_start_state_pub.publish(Empty())
