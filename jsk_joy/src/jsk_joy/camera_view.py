# camera_view.py
import math
import numpy
import tf
import rospy
from view_controller_msgs.msg import CameraPlacement

class CameraView():
  def __init__(self):
    self.yaw = 0.0
    self.pitch = 0.0
    #self.roll = 0.0
    self.distance = 2.0
    self.focus = numpy.array((0, 0, 0))
    self.z_up = numpy.array((0, 0, 1))
  def viewPoint(self):
    p = numpy.array((self.distance * math.cos(self.yaw) * math.cos(self.pitch) + self.focus[0],
                     self.distance * math.sin(self.yaw) * math.cos(self.pitch) + self.focus[1],
                     self.distance *                      math.sin(self.pitch) + self.focus[2]))
    return p
  def cameraOrientation(self):
    OE = self.viewPoint()
    f = self.focus - OE # z
    f = f / tf.transformations.vector_norm(f)
    u = numpy.array((0, 0, 1))            #not aligned y
    r = numpy.cross(u, f) # x
    r = r / tf.transformations.vector_norm(r)
    uy = numpy.cross(f, r)
    uy = uy / tf.transformations.vector_norm(uy)
    m = tf.transformations.identity_matrix()[:3, :3]   #rotation matrix
    m[0, 0] = r[0]
    m[1, 0] = r[1]
    m[2, 0] = r[2]
    m[0, 1] = uy[0]
    m[1, 1] = uy[1]
    m[2, 1] = uy[2]
    m[0, 2] = f[0]
    m[1, 2] = f[1]
    m[2, 2] = f[2]
    return m
  def cameraPlacement(self):
    #TIME = 0.05
    TIME = 1.0 / 40 * 2.0
    view_point = self.viewPoint()
    placement = CameraPlacement()
    placement.interpolation_mode = CameraPlacement.LINEAR
    placement.time_from_start = rospy.Duration(TIME)
    placement.eye.header.stamp = rospy.Time(0.0)
    placement.eye.point.x = view_point[0]
    placement.eye.point.y = view_point[1]
    placement.eye.point.z = view_point[2]
    placement.focus.header.stamp = rospy.Time(0.0)
    placement.focus.point.x = self.focus[0]
    placement.focus.point.y = self.focus[1]
    placement.focus.point.z = self.focus[2]
    placement.up.header.stamp = rospy.Time(0.0)
    placement.up.vector.x = self.z_up[0]
    placement.up.vector.y = self.z_up[1]
    placement.up.vector.z = self.z_up[2]
    placement.mouse_interaction_mode = CameraPlacement.ORBIT
    return placement
