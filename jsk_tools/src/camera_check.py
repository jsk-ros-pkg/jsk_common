#!/usr/bin/env python

from __future__ import division

import os
import collections
import fcntl

import rostopic
import rospy
import diagnostic_updater
import diagnostic_msgs
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sound_play.msg import SoundRequest

from jsk_tools.sanity_lib import checkUSBExist


class CameraCheck(object):

    def __init__(self, device_path=None):
        self.bridge = CvBridge()
        self.topic_names = rospy.get_param('~topic_names', [])
        self.device_type = rospy.get_param("~device_type", 'usb')
        self.device_path = rospy.get_param('~device_path', None)
        self.vendor_id = rospy.get_param('~vid', None)
        self.product_id = rospy.get_param('~pid', None)
        self.duration = rospy.get_param('~duration', 1)
        self.frequency = rospy.get_param('~frequency', None)
        self.speak_enabled = rospy.get_param("~speak", True)
        self.speak_pub = rospy.Publisher(
            "/robotsound", SoundRequest, queue_size=1)

        # nodelet manager name for restarting
        self.camera_nodelet_manager_name = rospy.get_name(
            "~camera_nodelet_manager_name", None)
        if self.camera_nodelet_manager_name is not None:
            self.camera_nodelet_manager_name = rospy.get_name(
                "~child_camera_nodelet_manager_name",
                os.path.basename(self.camera_nodelet_manager_name))
        self.restart_camera_command = rospy.get_param(
            '~restart_camera_command', None)

        self.diagnostic_updater = diagnostic_updater.Updater()
        self.diagnostic_updater.setHardwareID(rospy.get_param("~hardware_id", 'none'))
        self.diagnostic_updater.add('connection', self.check_connection)
        self.diagnostic_updater.add('image', self.check_topic)

        self._is_subscribing = False

    def subscribe(self):
        self.subs = []
        self.topic_msg_dict = {}
        for topic_name in self.topic_names:
            self.topic_msg_dict[topic_name] = []
            msg_class, _, _ = rostopic.get_topic_class(topic_name, blocking=True)
            sub = rospy.Subscriber(topic_name, msg_class,
                                   callback=lambda msg : self.callback(topic_name, msg),
                                   queue_size=1)
            self.subs.append(sub)
        self._is_subscribing = True

    def unsubscribe(self):
        if self._is_subscribing is False:
            return
        for sub in self.subs:
            sub.unregister()
        self._is_subscribing = False

    def speak(self, speak_str):
        rospy.logerr(
            "[%s] %s", self.__class__.__name__, speak_str)
        if self.speak_enabled:
            msg = SoundRequest()
            msg.sound = SoundRequest.SAY
            msg.command = SoundRequest.PLAY_ONCE
            msg.arg = speak_str
            self.speak_pub.publish(msg)

    def callback(self, topic_name, msg):
        self.topic_msg_dict[topic_name].append(msg)

    def check_connection(self, stat):
        if self.device_type == 'usb':
            if self.device_path is not None:
                if os.path.exists(self.device_path):
                    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                                 'device exists : {}'.format(self.device_path))
                else:
                    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                                 'device not exists : {}'.format(self.device_path))
            elif (self.vendor_id is not None) and (self.product_id is not None):
                if checkUSBExist(self.vendor_id, self.product_id):
                    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                                 'device exists : {}:{}'.format(
                                     self.vendor_id, self.product_id))
                else:
                    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                                 'device exists : {}:{}'.format(
                                     self.vendor_id, self.product_id))
        else:
            raise NotImplementedError("device_type {} is not yet supported".
                                      format(self.device_type))
        return stat

    def reset_usb(self):
        if self.device_path is None:
            rospy.logwarn('device_path is not exists. '
                          'Please set device_path')
            return False
        fd = os.open(self.device_path, os.O_WROMLY)
        if fd < 0:
            rospy.logerr("Could not open {}".format(self.device_path))
            return False
        rospy.loginfo("Resetting USB device")
        # Equivalent of the _IO('U', 20) constant in the linux kernel.
        USBDEVFS_RESET = ord('U') << (4*2) | 20
        try:
            rc = fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        finally:
            os.cloose(fd)

    def check_topic(self, stat):
        for topic_name in self.topic_msg_dict.keys():
            msgs = self.topic_msg_dict[topic_name]
            if len(msgs) == 0:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                             'topic {} not available'.format(topic_name))
            else:
                if self.frequency == None:
                    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                                 'topic {} available'.format(topic_name))
                else:
                    topic_hz = len(msgs) / self.duration
                    if topic_hz >= self.frequency:
                        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                                     'topic {} available'.format(topic_name))
                    else:
                        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                                     'topic {} available'.format(topic_name))
                    stat.add('topic {} {} Hz'.format(topic_name, topic_hz))
        return stat

    def is_image_valid(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            sum_of_pixels = max(cv2.sumElems(cv_image))
        except CvBridgeError as e:
            rospy.logerr(
                "[%s] failed to convert image to cv",
                self.__class__.__name__)
            return False
        rospy.loginfo(
            "[%s] sum of pixels is %d at %s",
            self.__class__.__name__,
            sum_of_pixels, msg.header.stamp.secs)
        if sum_of_pixels == 0:
            return False
        return True

    def run(self):
        while not rospy.is_shutdown():
            self.subscribe()
            rospy.sleep(self.duration)
            self.unsubscribe()
            self.diagnostic_updater.update()


def main():
    rospy.init_node('camera_check')
    cc = CameraCheck()
    cc.run()
    rospy.spin()


if __name__ == '__main__':
    main()
