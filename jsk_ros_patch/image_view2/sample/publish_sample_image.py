#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish an image file, its camera_info and image_view2/ImageMarker2s.

ROS 2 sample for image_view2 (see sample/sample_image_view2.launch.xml).

Publications:
  image (sensor_msgs/Image): ~file_name, every 1 / ~rate seconds
  camera_info (sensor_msgs/CameraInfo): a pinhole model of the image
  image_marker (image_view2/ImageMarker2): 2D markers drawn on the image
"""

import math
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from image_view2.msg import ImageMarker2
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def const(value):
    # the constants of ImageMarker2 are `byte`, bytes of one byte in rclpy,
    # while its type and action fields are int32
    return ord(value)


def point(x, y):
    return Point(x=float(x), y=float(y), z=0.0)


class PublishSampleImage(Node):

    def __init__(self):
        super().__init__('publish_sample_image')
        default_file = os.path.join(
            get_package_share_directory('image_view2'),
            'sample', 'kiva_pod_image_color.jpg')
        file_name = self.declare_parameter('file_name', default_file).value
        rate = self.declare_parameter('rate', 10.0).value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value

        img = cv2.imread(file_name)
        if img is None:
            raise RuntimeError('cannot read image: {}'.format(file_name))
        self.image_msg = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
        self.image_msg.header.frame_id = self.frame_id
        self.info_msg = self.camera_info(img.shape[1], img.shape[0])

        self.image_pub = self.create_publisher(Image, 'image', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.marker_pub = self.create_publisher(
            ImageMarker2, 'image_marker', 10)
        self.count = 0
        self.static_marker_period = max(1, int(round(rate)))
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

    def camera_info(self, width, height):
        # a pinhole camera of 60 deg horizontal field of view, for
        # 3D markers someone may add; the 2D markers below do not use it
        f = width / 2.0 / math.tan(math.radians(30))
        cx, cy = width / 2.0, height / 2.0
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def marker(self, marker_id, marker_type, stamp):
        marker = ImageMarker2()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'sample'
        marker.id = marker_id
        marker.type = const(marker_type)
        marker.action = const(ImageMarker2.ADD)
        return marker

    def static_markers(self, stamp):
        markers = []

        m = self.marker(0, ImageMarker2.TEXT, stamp)
        m.position = point(10, 30)
        m.left_up_origin = True
        m.text = 'image_view2 sample'
        m.scale = 0.8
        markers.append(m)

        # outline of the shelf
        m = self.marker(1, ImageMarker2.LINE_STRIP, stamp)
        m.points = [point(110, 175), point(425, 175), point(425, 355),
                    point(110, 355), point(110, 175)]
        m.outline_colors = [color(0, 1, 0)]
        m.width = 3.0
        markers.append(m)

        # the box on the shelf
        m = self.marker(2, ImageMarker2.POLYGON, stamp)
        m.points = [point(280, 215), point(310, 215), point(310, 272),
                    point(280, 272)]
        m.outline_colors = [color(1, 0, 0)]
        m.width = 3.0
        markers.append(m)

        m = self.marker(3, ImageMarker2.TEXT, stamp)
        m.position = point(295, 200)
        m.text = 'box'
        m.scale = 0.6
        markers.append(m)

        m = self.marker(4, ImageMarker2.POINTS, stamp)
        m.points = [point(180, 265), point(250, 265), point(370, 255),
                    point(300, 340), point(370, 330)]
        m.outline_colors = [color(1, 0, 1)]
        m.scale = 5.0
        markers.append(m)
        return markers

    def moving_marker(self, stamp):
        # a circle going round the shelf, to show the markers are redrawn
        th = self.count * 0.1
        m = self.marker(5, ImageMarker2.CIRCLE, stamp)
        m.position = point(267 + 120 * math.cos(th), 265 + 60 * math.sin(th))
        m.scale = 20.0
        m.width = 3.0
        m.outline_color = color(0, 0.5, 1)
        m.filled = b'\x01'  # byte
        m.fill_color = color(1, 1, 0)
        return m

    def timer_cb(self):
        stamp = self.get_clock().now().to_msg()
        self.image_msg.header.stamp = stamp
        self.info_msg.header.stamp = stamp
        self.image_pub.publish(self.image_msg)
        self.info_pub.publish(self.info_msg)
        # image_view2 redraws (and publishes image/marked) on every marker,
        # so the markers that do not change are sent only once a second
        # (not once: image_view2 may start after this node)
        if self.count % self.static_marker_period == 0:
            for m in self.static_markers(stamp):
                self.marker_pub.publish(m)
        self.marker_pub.publish(self.moving_marker(stamp))
        self.count += 1


def main():
    rclpy.init()
    node = PublishSampleImage()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
