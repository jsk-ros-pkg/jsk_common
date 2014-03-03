import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from jsk_footstep_msgs.msg import PlanFootstepsAction, PlanFootstepsGoal, Footstep, FootstepArray
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose
import tf
from tf.transformations import *
import numpy

def poseToMatrix(pose):
    mat = quaternion_matrix(numpy.array((pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w)))
                                         
    mat[0, 3] = pose.position.x
    mat[1, 3] = pose.position.y
    mat[2, 3] = pose.position.z
    return mat
def matrixToPose(mat):
    q = quaternion_from_matrix(mat)
    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    pose.position.x = mat[0, 3]
    pose.position.y = mat[1, 3]
    pose.position.z = mat[2, 3]
    return pose
    

class JoyFootstepPlannerDemo(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyFootstepPlannerDemo')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/map')
    self.lleg_frame_id = rospy.get_param('~lleg_frame_id', '/lfsensor')
    self.rleg_frame_id = rospy.get_param('~rleg_frame_id', '/rfsensor')
    self.br = tf.TransformBroadcaster()
    self.lleg_pose = Pose()
    self.lleg_pose.position.y = 0.1
    self.lleg_pose.orientation.w = 1.0
    self.rleg_pose = Pose()
    self.rleg_pose.position.y = -0.1
    self.rleg_pose.orientation.w = 1.0
    self.command_pub = rospy.Publisher('/menu_command', UInt8)
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    # broad cast tf
    self.br.sendTransform((self.lleg_pose.position.x, self.lleg_pose.position.y, self.lleg_pose.position.z),
                     (self.lleg_pose.orientation.x, self.lleg_pose.orientation.y, self.lleg_pose.orientation.z, self.lleg_pose.orientation.w),
                     rospy.Time.now(),
                     self.lleg_frame_id,
                     self.frame_id)
    self.br.sendTransform((self.rleg_pose.position.x, self.rleg_pose.position.y, self.rleg_pose.position.z),
                     (self.rleg_pose.orientation.x, self.rleg_pose.orientation.y, self.rleg_pose.orientation.z, self.rleg_pose.orientation.w),
                     rospy.Time.now(),
                     self.rleg_frame_id,
                     self.frame_id)

    latest = history.latest()
    if not latest:
      return
    if status.triangle and not latest.triangle:
      self.command_pub.publish(UInt8(1))
    elif status.cross and not latest.cross:
      self.command_pub.publish(UInt8(2))
    if status.circle and not latest.circle:
      base_mat = poseToMatrix(self.pre_pose.pose)
      lleg_offset = Pose()
      lleg_offset.position.y = 0.1
      lleg_offset.orientation.w = 1.0
      rleg_offset = Pose()
      rleg_offset.position.y = -0.1
      rleg_offset.orientation.w = 1.0
      
      left_offset_mat = poseToMatrix(lleg_offset)
      right_offset_mat = poseToMatrix(rleg_offset)
      new_lleg_mat = numpy.dot(base_mat, left_offset_mat)
      new_rleg_mat = numpy.dot(base_mat, right_offset_mat)
      self.lleg_pose = matrixToPose(new_lleg_mat)
      self.rleg_pose = matrixToPose(new_rleg_mat)
      # update the position of tf
      # self.pre_pose
    
