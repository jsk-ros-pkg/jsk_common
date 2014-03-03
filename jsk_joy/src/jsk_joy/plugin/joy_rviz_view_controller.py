# joy_rviz_view_controller

from jsk_joy.joy_plugin import JSKJoyPlugin
from jsk_joy.camera_view import CameraView
from view_controller_msgs.msg import CameraPlacement

import tf
import rospy

import numpy
import math

def signedSquare(val):
  if val > 0:
    sign = 1
  else:
    sign = -1
  return val * val * sign

class RVizViewController(JSKJoyPlugin):
  def __init__(self, name):
    JSKJoyPlugin.__init__(self, name)
    self.camera_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement)
    self.pre_view = CameraView()
    self.follow_view = rospy.get_param('~follow_view', False)
    self.counter = 0
  def joyCB(self, status, history):
    self.counter = self.counter + 1
    if self.counter > 1024:
      self.counter = 0
    if self.counter % 2 != 0:
      pass
    pre_view = self.pre_view
    view = CameraView()
    view.focus = numpy.copy(pre_view.focus)
    view.yaw = pre_view.yaw
    view.pitch = pre_view.pitch
    view.distance = pre_view.distance
    view_updated = False
    if status.R3:
      if not status.left_analog_y == 0.0:
        view.distance = view.distance - signedSquare(status.left_analog_y) * 0.05
        view_updated = True
      # calc camera orietation
      if status.left:
        view_updated = True
        view_x = 1.0
      elif status.right:
        view_updated = True
        view_x = -1.0
      else:
        view_x = 0.0
      if status.up:
        view_updated = True
        view_y = 1.0
      elif status.down:
        view_updated = True
        view_y = -1.0
      else:
        view_y = 0.0
      focus_diff = numpy.dot(view.cameraOrientation(),
                             numpy.array((view_x / 20.0 / view.distance,
                                          view_y / 20.0 / view.distance,
                                          0)))
      view.focus = view.focus + focus_diff
    else:
      if status.right_analog_x != 0.0:
        view_updated = True
      if status.right_analog_y != 0.0:
        view_updated = True
      
      view.yaw = view.yaw - 0.05 * signedSquare(status.right_analog_x)
      view.pitch = view.pitch + 0.05 * signedSquare(status.right_analog_y)

    if self.follow_view and self.support_follow_view:
      view_updated = True
      view.focus = numpy.array((self.pre_pose.pose.position.x,
                                self.pre_pose.pose.position.y,
                                self.pre_pose.pose.position.z))
      #view.yaw = math.pi
      q = numpy.array((self.pre_pose.pose.orientation.x,
                       self.pre_pose.pose.orientation.y,
                       self.pre_pose.pose.orientation.z,
                       self.pre_pose.pose.orientation.w))
      mat = tf.transformations.quaternion_matrix(q)
      camera_local_pos = numpy.dot(mat, numpy.array((0, 0, 1, 1)))[:3]
      pitch = math.asin(camera_local_pos[2])
      # calc pitch quadrant
      if camera_local_pos[1] < 0:
        pitch = math.pi - pitch
      if math.fabs(math.cos(pitch)) < 0.01:
        yaw = 0.0
      else:
        cos_value = camera_local_pos[0] / math.cos(pitch)
        if cos_value > 1.0:
          cos_value = 1.0
        elif cos_value < -1.0:
          cos_value = -1.0
        yaw = math.acos(cos_value)
      view.pitch = pitch
      view.yaw = yaw
      z_up = numpy.dot(mat, numpy.array((1, 0, 0, 1)))
      view.z_up = z_up[:3]
    if view_updated and self.counter % 10 == 0:
      placement = view.cameraPlacement()
      self.camera_pub.publish(placement)
    self.pre_view = view
