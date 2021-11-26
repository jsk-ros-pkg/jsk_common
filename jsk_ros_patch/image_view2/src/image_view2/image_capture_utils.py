# -*- encoding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

from image_view2.srv import Capture
from image_view2.srv import CaptureRequest
from image_view2.srv import CaptureResponse

from queue import Queue


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
        rospy.loginfo('Error: {}'.format(e))
        return False, 'Error: {}'.format(e)


class ImageCaptureServer:

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.task_queue = Queue()
        self.srv = rospy.Service(
                        '~capture',
                        Capture,
                        self.handler)

    def spin(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.task_queue.empty():
                req = self.task_queue.get()
                success, message = capture_image(
                                req.image_topic,
                                req.file_name,
                                self.cv_bridge
                                )
                rospy.loginfo('Capture a image: {}, {}'.format(success, message))

    def handler(self, req):
        self.task_queue.put(req)
        res = CaptureResponse()
        res.success = True
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
