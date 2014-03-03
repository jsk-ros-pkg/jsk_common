import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from jsk_footstep_msgs.msg import PlanFootstepsAction, PlanFootstepsGoal, Footstep, FootstepArray
from std_msgs.msg import UInt8

class JoyFootstepPlanner(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyFootstepPlanner')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/map')
    self.command_pub = rospy.Publisher('/menu_command', UInt8)
  def joyCB(self, status, history):    
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.triangle and not latest.triangle:
      self.command_pub.publish(UInt8(1))
    elif status.cross and not latest.cross:
      self.command_pub.publish(UInt8(2))
    
