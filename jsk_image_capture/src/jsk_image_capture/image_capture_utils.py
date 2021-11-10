# -*- encoding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

from jsk_image_capture.srv import Capture
from jsk_image_capture.srv import CaptureRequest
from jsk_image_capture.srv import CaptureResponse


def capture_image(image_topic, file_name, cv_bridge):
    duration_timeout = rospy.get_param('~duration_timeout', 10.0)
    try:
        msg = rospy.wait_for_message(
                        image_topic,
                        Image,
                        timeout=rospy.Duration(duration_timeout)
                        )
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(file_name, image)
        return True, 'Success'
    except rospy.ROSException as e:
        rospy.loginfo('Error: {]'.format(e))
        return False, 'Error: {}'.format(e)


class ImageCaptureServer:

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.srv = rospy.Service(
                        '~capture',
                        Capture,
                        self.handler)

    def handler(self, req):
        success, message = capture_image(
                                req.image_topic,
                                req.file_name,
                                self.cv_bridge
                                )
        res = CaptureResponse()
        res.success = success
        res.message = message
        return res


class ImageCaptureClient:

    def __init__(self):

        rospy.wait_for_service(
                '/image_capture_server/capture',
                rospy.Duration(10)
                )
        self.client = rospy.ServiceProxy(
                '/image_capture_server/capture',
                Capture
                )

    def capture(self,
                image_topic,
                file_name
                ):
        req = CaptureRequest()
        req.image_topic = image_topic
        req.file_name = file_name
        ret = self.client(req)
        return ret.success, ret.message
