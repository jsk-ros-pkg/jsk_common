from geometry_msgs.msg import PoseStamped, Pose
from joy_pose_6d import JoyPose6D
from jsk_footstep_msgs.msg import FootstepArray, Footstep
import tf
import rospy

class FootstepCoords():
  def __init__(self, position=[0,0,0], angles=[0,0,0], leg=0, pose=None):
    self.position = position
    self.angles = angles
    self.leg = leg
  def toROSPose(self):
    pose = Pose()
    pose.position.x = self.position[0]
    pose.position.y = self.position[1]
    pose.position.z = self.position[2]
    q = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose
  def toROSMsg(self):
    footstep = Footstep()
    footstep.leg = self.leg
    footstep.pose = self.toROSPose()
    return footstep

def FootstepCoordsToROSMsg(footsteps):
  pass

class JoyFootstep(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyFootstep', publish_pose=False)
    self.support_follow_view = True
    self.footstep_pub = rospy.Publisher('/footstep', FootstepArray)
    self.footsteps = []
    self.frame_id = rospy.get_param('~frame_id', '/map')
    
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    footsteps = FootstepArray()
    footsteps.header.frame_id = self.frame_id
    footsteps.header.stamp = rospy.Time(0.0)
    
    if status.triangle and not history.latest().triangle:
      # remove the latest one
      if len(self.footsteps) >= 2:
        self.footsteps = self.footsteps[:-1]
        self.pre_pose.pose = self.footsteps[-1].pose
      elif len(self.footsteps) == 1:
        self.footsteps = []
      if len(self.footsteps) == 0:
        self.pre_pose.pose.position.x = 0
        self.pre_pose.pose.position.y = 0
        self.pre_pose.pose.position.z = 0
        self.pre_pose.pose.orientation.x = 0
        self.pre_pose.pose.orientation.y = 0
        self.pre_pose.pose.orientation.z = 0
        self.pre_pose.pose.orientation.w = 1
    # pre_pose -> Footstep
    current_step = Footstep()
    current_step.pose = self.pre_pose.pose
    if status.cross and not history.latest().cross:
      # left
      current_step.leg = Footstep.LEFT
      self.footsteps.append(current_step)
    elif status.circle and not history.latest().circle:
      # right
      current_step.leg = Footstep.RIGHT
      self.footsteps.append(current_step)
    
        
    footsteps.footsteps.extend(self.footsteps)
    footsteps.footsteps.append(current_step)
    self.footstep_pub.publish(footsteps)
