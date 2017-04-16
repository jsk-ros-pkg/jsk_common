#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.msg import ImageRecognitionCandidate
from docomo_perception.srv import (
    ImageRecognition,
    ImageRecognitionRequest,
    ImageRecognitionResponse
)
from cv_bridge import CvBridge, CvBridgeError
import cv2

import json
import requests
import yaml
import threading
import tempfile

global APIHOST, APIKEY
settings_path = "/var/lib/robot/docomo_image_recognition_settings.yaml"


class DocomoImageRecognitionNode(object):
    def __init__(self):
        self.srv = rospy.Service("image_recognition", ImageRecognition, self.serviceCallback)
        self.lock = threading.Lock()
        self.bridge = CvBridge()

    def _save_jpg_from_imgmsg(self, msg):
        mat = self.bridge.imgmsg_to_cv2(msg)
        img_path = tempfile.mktemp(prefix='docomo_img_recog_',suffix='.jpg')
        cv2.imwrite(img_path, mat)
        return img_path

    def _type(self, t):
        if t == ImageRecognitionRequest.ALL:
            return "product-all"
        elif t == ImageRecognitionRequest.BOOK:
            return "book"
        elif t == ImageRecognitionRequest.CD:
            return "cd"
        elif t == ImageRecognitionRequest.DVD:
            return "dvd"
        elif t == ImageRecognitionRequest.GAME:
            return "game"
        elif t == ImageRecognitionRequest.SOFTWARE:
            return "software"
        elif t == ImageRecognitionRequest.FOOD:
            return "food"
        else:
            rospy.logerr("invalid type error")
            return "product-all"
        
    def serviceCallback(self, req):
        jpg_path = self._save_jpg_from_imgmsg(req.image)
        jpg_data = None
        with open(jpg_path, 'rb') as f:
            jpg_data = f.read()

        params = {
            "APIKEY": APIKEY,
            "recog": self._type(req.type),
            "numOfCandidates": str(req.numOfCandidates)
            }
        headers = {
            "Content-type": "application/octet-stream"
            }
        self.lock.acquire()
        urlres = requests.post(APIHOST,
                               data=jpg_data,
                               headers=headers,
                               params=params)
        self.lock.release()
        rospy.loginfo("sent request: %d", urlres.status_code)
        if urlres.status_code is not requests.codes.ok:
            rospy.logerr("failed to send request: %d", urlres.status_code)
            return ImageRecognitionResponse()

        resdic = json.loads(urlres.content)

        res = ImageRecognitionResponse()
        candidates = []
        if resdic.has_key('candidates') and len(resdic["candidates"]) > 0:
            res.candidates = [ ImageRecognitionCandidate(float(c["score"]),
                                                         c["category"],
                                                         c["detail"]["itemName"],
                                                         c["imageUrl"]) for c in resdic["candidates"] ]
        return res

def load_image_recognition_settings():
    global APIHOST, APIKEY

    try:
        with open(settings_path) as f:
            key = yaml.load(f)
            APIHOST = key['APIHOST']
            APIKEY = key['APIKEY']
            rospy.loginfo("loaded settings")
    except IOError as e:
        rospy.logerr('"%s" not found : %s' % (settings_path, e))
        exit(-1)
    except Exception as e:
        rospy.logerr("failed to load settings: %s", e)
        rospy.logerr("check if exists valid setting file in %s", settings_path)
        exit(-1)

if __name__ == '__main__':
    rospy.init_node("docomo_image_recognition_node")
    n = DocomoImageRecognitionNode()
    load_image_recognition_settings()
    rospy.spin()
