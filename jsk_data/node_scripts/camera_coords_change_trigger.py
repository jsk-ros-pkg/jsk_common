#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION


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

    def __init__(self, node=None):
        self.node = node
        self.delta_pos = ros_compat.get_param(node, '~delta_position', 0.03)
        self.delta_ori = ros_compat.get_param(node, '~delta_orientation', 0.03)
        self.velocify_pos = ros_compat.get_param(node, '~velocify_position', 0.01)
        self.velocify_ori = ros_compat.get_param(node, '~velocify_orientation', 0.01)

        ros_compat.loginfo(node, "Waiting for service 'trigger'")
        self.tf_buffer, self.listener = ros_compat.create_tf_buffer_and_listener(node)
        if ROS_VERSION == 2:
            self.trigger_client = node.create_client(Trigger, '~/trigger')
            self.trigger_client.wait_for_service()
        else:
            ros_compat.rospy.wait_for_service('~trigger')
            self.trigger = ros_compat.rospy.ServiceProxy('~trigger', Trigger)
        self.timer = ros_compat.create_timer(node, 1. / 100, self.timer_callback)
        self.last_pose_stamped = None
        self.last_saved_pose_stamped = None

    def _lookup_camera_pose(self, src_frame, dst_frame, stamp):
        transform = self.tf_buffer.lookup_transform(
            src_frame, dst_frame, stamp, timeout=ros_compat.duration_from_sec(1.0))
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = src_frame
        pose_stamped.header.stamp = stamp.to_msg() if ROS_VERSION == 2 else stamp
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped

    def timer_callback(self, event=None):
        stamp = self.node.get_clock().now() if ROS_VERSION == 2 else ros_compat.rospy.Time.now()
        src_frame = 'world' if ROS_VERSION == 2 else '/world'
        dst_frame = 'head_mount_kinect_rgb_optical_frame' if ROS_VERSION == 2 \
            else '/head_mount_kinect_rgb_optical_frame'
        try:
            pose_stamped = self._lookup_camera_pose(src_frame, dst_frame, stamp)
        except Exception as e:
            ros_compat.logerr(self.node, str(e))
            return

        # Check the change
        delta_saving_pos, delta_saving_ori = np.inf, np.inf
        vel_pos, vel_ori = 0, 0
        if self.last_pose_stamped is not None:
            delta_pos, delta_ori = compute_pose_delta(
                pose_stamped.pose, self.last_pose_stamped.pose)
            delta_time = ros_compat.stamp_to_sec(pose_stamped.header.stamp) - \
                ros_compat.stamp_to_sec(self.last_pose_stamped.header.stamp)
            if delta_time > 0:
                vel_pos = delta_pos / delta_time
                vel_ori = delta_ori / delta_time
        if self.last_saved_pose_stamped is not None:
            delta_saving_pos, delta_saving_ori = compute_pose_delta(
                pose_stamped.pose, self.last_saved_pose_stamped.pose)

        self.last_pose_stamped = pose_stamped

        logging_msg = ('delta_saving_pos: {}, delta_saving_ori: {}, '
                       'vel_pos: {}, vel_ori: {}'
                       .format(delta_saving_pos,
                               delta_saving_ori, vel_pos, vel_ori))
        if (delta_saving_pos > self.delta_pos and
                delta_saving_ori > self.delta_ori and
                vel_pos < self.velocify_pos and
                vel_ori < self.velocify_ori):
            ros_compat.loginfo(self.node, logging_msg)
            ros_compat.loginfo(self.node, 'Sending saving result')
            if ROS_VERSION == 2:
                future = self.trigger_client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f: self._on_trigger_result(f.result(), pose_stamped))
            else:
                res = self.trigger()
                self._on_trigger_result(res, pose_stamped)
        else:
            ros_compat.loginfo(self.node, logging_msg)

    def _on_trigger_result(self, res, pose_stamped):
        ros_compat.loginfo(
            self.node, 'Saving request result success: {}'.format(res.success))
        if res.success:
            self.last_saved_pose_stamped = pose_stamped


def main():
    if ROS_VERSION == 2:
        ros_compat.rclpy.init()
        node = ros_compat.rclpy.create_node('camera_coords_change_trigger')
        CameraCoordsChangeTrigger(node)
        ros_compat.spin_and_shutdown(node)
    else:
        ros_compat.rospy.init_node('camera_coords_change_trigger')
        CameraCoordsChangeTrigger()
        ros_compat.rospy.spin()


if __name__ == '__main__':
    main()
