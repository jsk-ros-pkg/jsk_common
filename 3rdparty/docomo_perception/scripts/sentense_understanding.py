#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.srv import (SentenseUnderstanding, SentenseUnderstandingResponse)

import json
import requests
import yaml
import threading

global APIHOST, APIKEY, APPKEY, CLIENTVER
settings_path = "/var/lib/robot/docomo_sentense_understanding_settings.yaml"


class DocomoSentenseUnderstandingNode(object):
    def __init__(self):
        self.srv = rospy.Service("sentense_understanding", SentenseUnderstanding, self.serviceCallback)
        self.lock = threading.Lock()

    def serviceCallback(self, req):
        dic = {
            "projectKey": "OSU",
            "appInfo": {
                "appKey": APPKEY
            },
            "clientVer": CLIENTVER,
            "language": "ja",
            "userUtterance": {
                "utteranceText": req.text
            }
        }
        sendData = json.dumps(dic, sort_keys=False, ensure_ascii=False, indent=2)
        sendData = "json=" + sendData
        rospy.loginfo("sendData: %s", sendData)
        params = {
            "APIKEY": APIKEY
        }
        headers = {
            "Content-type": "application/x-www-form-urlencoded"
        }
        
        self.lock.acquire()
        urlres = requests.post(APIHOST, params=params, headers=headers, data=sendData)
        self.lock.release()

        rospy.loginfo("sent request %s -> %d", urlres.url, urlres.status_code)
        rospy.loginfo("content: %s", urlres.content)
        if urlres.status_code is not requests.codes.ok:
            rospy.logerr("bad status code: %d", urlres.status_code)
            return SentenseUnderstandingResponse()
        resdic = json.loads(urlres.content)
        res = SentenseUnderstandingResponse()
        res.command = resdic["dialogStatus"]["command"]["commandName"]
        res.revisedText = resdic["userUtterance"]["utteranceRevised"]
        res.words = resdic["userUtterance"]["utteranceWord"]
        if resdic["dialogStatus"].has_key("slotStatus"):
            res.args = [ s["slotValue"] for s in resdic["dialogStatus"]["slotStatus"] ]
        res.extractedWords = [ s['wordsValue'] for s in resdic['extractedWords'] ]
        return res

def load_sentense_understanding_settings():
    global APIHOST, APIKEY, APPKEY, APPKEY, CLIENTVER

    try:
        with open(settings_path) as f:
            key = yaml.load(f)
            APIHOST = key['APIHOST']
            APIKEY = key['APIKEY']
            APPKEY = key['APPKEY']
            CLIENTVER = key['CLIENTVER']
            rospy.loginfo("loaded settings")
    except IOError as e:
        rospy.logerr('"%s" not found : %s' % (settings_path, e))
        exit(-1)
    except Exception as e:
        rospy.logerr("failed to load settings: %s", e)
        rospy.logerr("check if exists valid setting file in %s", settings_path)
        exit(-1)

if __name__ == '__main__':
    rospy.init_node("docomo_sentense_understanding")
    n = DocomoSentenseUnderstandingNode()
    load_sentense_understanding_settings()
    rospy.spin()
