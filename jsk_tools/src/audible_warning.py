#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import defaultdict
from threading import Event
from threading import Lock
from threading import Thread

import actionlib
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rospy
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal

from jsk_tools.diagnostics_utils import filter_diagnostics_status_list


class SpeakThread(Thread):

    def __init__(self, rate=1.0, wait=True, blacklist=None,
                 language='en', wait_speak_duration_time=10):
        super(SpeakThread, self).__init__()
        self.wait_speak_duration_time = wait_speak_duration_time
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.lock = Lock()
        self.status_list = []
        self.history = defaultdict(lambda: 10)
        self.blacklist = blacklist
        self.language = language

        self.talk = actionlib.SimpleActionClient(
            "/robotsound", SoundRequestAction)
        self.talk.wait_for_server()

    def stop(self):
        self.event.set()

    def add(self, status_list):
        with self.lock:
            self.status_list = filter_diagnostics_status_list(
                status_list,
                self.blacklist)

    def pop(self):
        with self.lock:
            for status in self.status_list:
                if status.name not in self.history.keys():
                    self.history[status.name] += 1
                    return status
            return None

    def sweep(self):
        for k in list(self.history.keys()):
            self.history[k] -= 1
            if self.history[k] == 0:
                del self.history[k]

    def run(self):
        while not self.event.wait(self.rate):
            e = self.pop()
            if e:
                # rospy.loginfo("audible warning talking: %s" % e)
                sentence = e.name + ' ' + e.message
                sentence = sentence.replace('/', '   ')
                sentence = sentence.replace('_', '   ')

                goal = SoundRequestGoal()
                goal.sound_request.sound = SoundRequest.SAY
                goal.sound_request.command = SoundRequest.PLAY_ONCE
                goal.sound_request.arg = sentence
                goal.sound_request.volume = 1.0

                self.talk.send_goal(goal)
                if self.wait:
                    self.talk.wait_for_result(
                        rospy.Duration(self.wait_speak_duration_time))
            self.sweep()


class AudibleWarning(object):
    def __init__(self):
        self.history = []
        self.error = []
        self.stale = []

        speak_rate = rospy.get_param("~speak_rate", 1.0)
        speak_wait = rospy.get_param("~speak_wait", True)
        self.warn_stale = rospy.get_param('~warn_stale', False)
        blacklist = rospy.get_param("~blacklist", [])
        self.speak_thread = SpeakThread(speak_rate, speak_wait, blacklist)

        # run-stop
        self.run_stop = False

        # diag
        self.sub_diag = rospy.Subscriber("/diagnostics_agg", DiagnosticArray,
                                         self.diag_cb, queue_size=1)
        self.speak_thread.start()

    def on_shutdown(self):
        self.speak_thread.stop()
        self.speak_thread.join()

    def diag_cb(self, msg):
        if self.run_stop:
            return

        error = filter(lambda s: s.level == DiagnosticStatus.ERROR, msg.status)
        if self.warn_stale:
            stale = filter(lambda s: s.level == DiagnosticStatus.STALE,
                           msg.status)
        else:
            stale = []
        self.speak_thread.add(stale, error)


if __name__ == '__main__':
    rospy.init_node("audible_warning")
    aw = AudibleWarning()  # NOQA
    rospy.on_shutdown(aw.on_shutdown)
    rospy.spin()
