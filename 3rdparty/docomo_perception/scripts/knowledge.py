#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.srv import (Knowledge, KnowledgeResponse)

import json
import requests
import yaml
import threading

global APIHOST, APIKEY
setting_path = "/var/lib/robot/docomo_knowledge_settings.yaml"


class DocomoKnowledgeQANode(object):
    def __init__(self):
        self.srv = rospy.Service("knowledge", Knowledge, self.serviceCallback)
        self.lock = threading.Lock()

    def serviceCallback(self, req):
        params = {
            "APIKEY" : APIKEY,
            "q": req.question
            }
        self.lock.acquire()
        urlres = requests.get(APIHOST, params=params)
        self.lock.release()
        rospy.loginfo("sent request to %s -> %d", urlres.url, urlres.status_code)
        rospy.loginfo("content: %s", urlres.content)
        if urlres.status_code is not requests.codes.ok:
            rospy.logerr("bad status code: %d", urlres.status_code)
            return KnowledgeResponse()
        resdic = json.loads(urlres.content)
        res = KnowledgeResponse()
        if resdic['message']:
            res.answer = resdic['message']['textForDisplay']
            res.answerForSpeech = resdic['message']['textForSpeech']
        return res

def load_knowledege_settings():
    global APIHOST, APIKEY

    try:
        with open(setting_path) as f:
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
    rospy.init_node("docomo_knowledge_node")
    n = DocomoKnowledgeQANode()
    load_knowledege_settings()
    rospy.spin()
