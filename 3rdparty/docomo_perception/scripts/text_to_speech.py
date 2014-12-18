#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from docomo_perception.msg import TextToSpeech
from sound_play.msg import SoundRequest

import json
import requests
import yaml
import tempfile
import threading

global APIHOST, APIKEY
settings_path = "/var/lib/robot/docomo_tts_settings.yaml"

class DocomoTextToSpeechNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber("text_to_speech", TextToSpeech, self.callback)
        self.pub = rospy.Publisher("robotsound", SoundRequest)
        self.lock = threading.Lock()

    def _speakerID(self, m):
        if not m or m == TextToSpeech.FEMALE:
            return str(0)
        elif m == TextToSpeech.MALE:
            return str(1)

    def _speechRate(self, m):
        if not m: m = 1.00 # default
        if m < 0.50: m = 0.50
        elif m > 10.00: m = 10.00
        return "%.2f" % m

    def _powerRate(self, m):
        if not m: m = 1.00 # default
        if m < 0.00: m = 0.00
        elif m > 5.00: m = 5.00
        return "%.2f" % m

    def _voiceType(self, m):
        if not m: m = 0
        elif m < -2: m = -2
        elif m > 2: m = 2
        return str(m)

    def callback(self, msg):
        dic = {
            "Command": "AP_Synth",
            "TextData": msg.text,
            "SpeakerID": self._speakerID(msg.gender),
            "SpeechRate": self._speechRate(msg.speed),
            "PowerRate": self._powerRate(msg.power),
            "VoiceType": self._voiceType(msg.type),
            "AudioFileFormat": "1"  # AAC
            }
        sendData = json.dumps(dic)
        url = "%s?APIKEY=%s" % (APIHOST, APIKEY)
        headers = { 'Content-type': 'application/json' }
        self.lock.acquire()
        urlres = requests.post(url, data=sendData, headers=headers)
        self.lock.release()
        rospy.loginfo("sent request %s -> %d", msg.text, urlres.status_code)

        if urlres.status_code is not requests.codes.ok:
            rospy.logerr("failed to send request: %d", urlres.status_code)
            return

        voice_path = tempfile.mktemp(prefix='docomo_tts_')
        with open(voice_path, 'w') as f:
            f.write(urlres.content)
        rospy.loginfo("received voice data: %s" % voice_path)

        # command play sound
        pub_msg = SoundRequest()
        pub_msg.sound = SoundRequest.PLAY_FILE
        pub_msg.command = SoundRequest.PLAY_ONCE
        pub_msg.arg = voice_path
        self.pub.publish(pub_msg)


def load_tts_settings():
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
    rospy.init_node("docomo_tts_node")
    t = DocomoTextToSpeechNode()
    load_tts_settings()
    rospy.spin()
