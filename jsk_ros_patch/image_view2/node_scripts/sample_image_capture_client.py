#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from image_view2.image_capture_utils import ImageCaptureClient


def main():

    rospy.init_node('sample_image_capture_client')
    image_topic = rospy.get_param('~image_topic')
    directory = rospy.get_param('~directory', '/tmp')
    client = ImageCaptureClient()
    rate = rospy.Rate(1)
    for index in range(10):
        rate.sleep()
        file_name = directory + '/sample_image_{}.jpg'.format(index)
        client.capture(image_topic, file_name)
        rospy.loginfo('Capturing image from {} to {}'.format(image_topic, file_name))


if __name__ == '__main__':
    main()
