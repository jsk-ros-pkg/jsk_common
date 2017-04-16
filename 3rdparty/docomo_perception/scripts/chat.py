#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from docomo_perception.srv import (Chat, ChatResponse)

import json
import requests
import yaml
import threading

global APIHOST, APIKEY, NICKNAME, NICKNAME_Y, SEX, BLOODTYPE, BIRTHDATEY, BIRTHDATEM, BIRTHDATED, AGE, CONSTELLATIONS, PLACE
settings_path = "/var/lib/robot/docomo_chat_settings.yaml"


class DocomoChatNode(object):
    def __init__(self):
        self.srv = rospy.Service("chat", Chat, self.serviceCallback)
        self.lock = threading.Lock()

        self.context = None # this is used like api session id

    def serviceCallback(self, req):
        dic = {
                "utt": req.text,
                "nickname": NICKNAME,
                "nickname_y": NICKNAME_Y,
                "sex": SEX,
                "bloodtype": BLOODTYPE,
                "birthdateY": BIRTHDATEY,
                "birthdateM": BIRTHDATEM,
                "birthdateD": BIRTHDATED,
                "age": AGE,
                "constellations": CONSTELLATIONS,
                "place": PLACE,
                "mode": "dialog"
                }
        if self.context:
            dic['context'] = self.context
            rospy.loginfo("context: %s", self.context)
        sendData = json.dumps(dic)
        url = "%s?APIKEY=%s" % (APIHOST, APIKEY)
        self.lock.acquire() # lock because of API limit (max simultaneous connection: 1)
        urlres = requests.post(url, data=sendData)
        self.lock.release()
        rospy.loginfo("sent request %s -> %d", req.text, urlres.status_code)
        rospy.loginfo("content: %s", urlres.content)
        if urlres.status_code is not requests.codes.ok:
            rospy.logerr("bad status code: %d", urlres.status_code)
            return ChatResponse()
        resdic = json.loads(urlres.content)
        res = ChatResponse()
        res.text = resdic['utt']
        self.context = resdic['context']
        rospy.loginfo("received %s %s", res.text, self.context)
        return res

def load_chat_settings():
    global APIHOST, APIKEY, NICKNAME, NICKNAME_Y, SEX, BLOODTYPE, BIRTHDATEY, BIRTHDATEM, BIRTHDATED, AGE, CONSTELLATIONS, PLACE

    try:
        with open(settings_path) as f:
            key = yaml.load(f)
            APIHOST = key['APIHOST']
            APIKEY = key['APIKEY']
            NICKNAME = key['NICKNAME']
            NICKNAME_Y = key['NICKNAME_Y']
            SEX = key['SEX']
            BLOODTYPE = key['BLOODTYPE']
            BIRTHDATEY = key['BIRTHDATEY']
            BIRTHDATEM = key['BIRTHDATEM']
            BIRTHDATED = key['BIRTHDATED']
            AGE = key['AGE']
            CONSTELLATIONS = key['CONSTELLATIONS']
            PLACE = key['PLACE']
            rospy.loginfo("loaded settings")
    except IOError as e:
        rospy.logerr('"%s" not found : %s' % (settings_path, e))
        exit(-1)
    except Exception as e:
        rospy.logerr("failed to load settings: %s", e)
        rospy.logerr("check if exists valid setting file in %s", settings_path)
        exit(-1)


if __name__ == '__main__':
    rospy.init_node("docomo_chat_node")
    n = DocomoChatNode()
    load_chat_settings()
    rospy.spin()
