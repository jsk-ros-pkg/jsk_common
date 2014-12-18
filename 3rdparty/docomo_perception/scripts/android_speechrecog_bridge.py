#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from jsk_gui_msgs.msg import VoiceMessage
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class Android2SpeechRecognitionBridgeNode(object):
    def __init__(self):
        self.pub = rospy.Publisher("voice", SpeechRecognitionCandidates)
        self.sub = rospy.Subscriber("/Tablet/voice", VoiceMessage, self.androidCallback)

    def androidCallback(self, msg):
        self.pub.publish(SpeechRecognitionCandidates(transcript=msg.texts))


if __name__ == '__main__':
    rospy.init_node("android2speech_recognition")
    r = Android2SpeechRecognitionBridgeNode()
    rospy.spin()
