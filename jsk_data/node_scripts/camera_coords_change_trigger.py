#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from termcolor import colored

from geometry_msgs.msg import PoseStamped
import rospy
from std_srvs.srv import Trigger
import tf


def compute_pose_delta(pose1, pose2):
    delta_x = pose1.position.x - pose2.position.x
    delta_y = pose1.position.y - pose2.position.y
    delta_z = pose1.position.z - pose2.position.z
    delta_pos = np.linalg.norm([delta_x, delta_y, delta_z])
    delta_x = pose1.orientation.x - pose2.orientation.x
    delta_y = pose1.orientation.y - pose2.orientation.y
    delta_z = pose1.orientation.z - pose2.orientation.z
    delta_w = pose1.orientation.w - pose2.orientation.w
    delta_ori = np.linalg.norm([delta_x, delta_y, delta_z, delta_w])
    return delta_pos, delta_ori


class CameraCoordsChangeTrigger(object):

    def __init__(self):
        self.delta_pos = rospy.get_param('~delta_position', 0.03)
        self.delta_ori = rospy.get_param('~delta_orientation', 0.03)
        self.velocify_pos = rospy.get_param('~velocify_position', 0.01)
        self.velocify_ori = rospy.get_param('~velocify_orientation', 0.01)

        self.listener = tf.TransformListener()
        rospy.loginfo("Waiting for service '{}'"
                      .format(rospy.resolve_name('~trigger')))
        rospy.wait_for_service('~trigger')
        self.trigger = rospy.ServiceProxy('~trigger', Trigger)
        self.timer = rospy.Timer(rospy.Duration(1. / 100),
                                 self.timer_callback)
        self.last_pose_stamped = None
        self.last_saved_pose_stamped = None

    def timer_callback(self, event):
        # Get current camera coords
        stamp = rospy.Time.now()
        src_frame = '/world'
        dst_frame = '/head_mount_kinect_rgb_optical_frame'
        try:
            self.listener.waitForTransform(src_frame, dst_frame, stamp,
                                           timeout=rospy.Duration(1))
        except Exception as e:
            rospy.logerr(e)
            return
        dst_pose = self.listener.lookupTransform(src_frame, dst_frame, stamp)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = src_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose.position.x = dst_pose[0][0]
        pose_stamped.pose.position.y = dst_pose[0][1]
        pose_stamped.pose.position.z = dst_pose[0][2]
        pose_stamped.pose.orientation.x = dst_pose[1][0]
        pose_stamped.pose.orientation.y = dst_pose[1][1]
        pose_stamped.pose.orientation.z = dst_pose[1][2]
        pose_stamped.pose.orientation.w = dst_pose[1][3]

        # Check the change
        delta_saving_pos, delta_saving_ori = np.inf, np.inf
        vel_pos, vel_ori = 0, 0
        if self.last_pose_stamped is not None:
            delta_pos, delta_ori = compute_pose_delta(
                pose_stamped.pose, self.last_pose_stamped.pose)
            delta_time = stamp - self.last_pose_stamped.header.stamp
            vel_pos = delta_pos / delta_time.to_sec()
            vel_ori = delta_ori / delta_time.to_sec()
        if self.last_saved_pose_stamped is not None:
            delta_saving_pos, delta_saving_ori = compute_pose_delta(
                pose_stamped.pose, self.last_saved_pose_stamped.pose)

        self.last_pose_stamped = pose_stamped

        logging_msg = ('delta_saving_pos: {}, delta_saving_ori: {}, '
                       'vel_pos: {}, vel_ori: {}'
                       .format(delta_saving_pos,
                               delta_saving_ori, vel_pos, vel_ori))
        # Save according to the change
        if (delta_saving_pos > self.delta_pos and
                delta_saving_ori > self.delta_ori and
                vel_pos < self.velocify_pos and
                vel_ori < self.velocify_ori):
            rospy.loginfo(logging_msg)
            rospy.loginfo(colored('Sending saving result', 'blue'))
            res = self.trigger()
            rospy.loginfo(colored('Saving request result success: {}'
                                  .format(res.success),
                                  'green' if res.success else 'red'))
            if res.success:
                self.last_saved_pose_stamped = pose_stamped
        else:
            rospy.loginfo_throttle(1, logging_msg)


def main():
    rospy.init_node('camera_coords_change_trigger')
    CameraCoordsChangeTrigger()
    rospy.spin()


if __name__ == '__main__':
    main()
