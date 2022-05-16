#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import defaultdict
import heapq
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
                 language='en',
                 volume=1.0,
                 speak_interval=0,
                 wait_speak_duration_time=10):
        super(SpeakThread, self).__init__()
        self.wait_speak_duration_time = wait_speak_duration_time
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.volume = volume
        self.lock = Lock()
        self.status_list = []
        self.speak_interval = speak_interval
        tm = rospy.Time.now().to_sec() - speak_interval
        self.previous_spoken_time = defaultdict(lambda tm=tm: tm)
        self.blacklist = blacklist
        self.language = language

        self.talk = actionlib.SimpleActionClient(
            "/robotsound", SoundRequestAction)
        self.talk.wait_for_server()

    def stop(self):
        self.event.set()

    def add(self, status_list):
        with self.lock:
            status_list = filter_diagnostics_status_list(
                status_list, self.blacklist)
            for status in status_list:
                heapq.heappush(
                    self.status_list,
                    (rospy.Time.now().to_sec(), status))

    def pop(self):
        with self.lock:
            while len(self.status_list) > 0:
                _, status = heapq.heappop(self.status_list)
                if rospy.Time.now().to_sec() \
                        - self.previous_spoken_time[status.name] \
                        < self.speak_interval:
                    continue
                return status
            return None

    def run(self):
        while not self.event.wait(self.rate):
            e = self.pop()
            if e:
                if e.level == DiagnosticStatus.WARN:
                    prefix = 'warning.'
                elif e.level == DiagnosticStatus.ERROR:
                    prefix = 'error.'
                elif e.level == DiagnosticStatus.STALE:
                    prefix = 'stale.'
                else:
                    prefix = 'ok.'
                sentence = prefix + e.name + ' ' + e.message
                sentence = sentence.replace('/', ' ')
                sentence = sentence.replace('_', ' ')
                rospy.loginfo("audible warning talking: %s" % sentence)

                goal = SoundRequestGoal()
                goal.sound_request.sound = SoundRequest.SAY
                goal.sound_request.command = SoundRequest.PLAY_ONCE
                goal.sound_request.arg = sentence
                goal.sound_request.volume = self.volume

                self.previous_spoken_time[e.name] = rospy.Time.now().to_sec()
                self.talk.send_goal(goal)
                if self.wait:
                    self.talk.wait_for_result(
                        rospy.Duration(self.wait_speak_duration_time))


class AudibleWarning(object):

    def __init__(self):
        speak_rate = rospy.get_param("~speak_rate", 1.0)
        speak_interval = rospy.get_param("~speak_interval", 120.0)
        wait_speak = rospy.get_param("~wait_speak", True)
        volume = rospy.get_param("~volume", 1.0)
        language = rospy.get_param('~language', 'en')

        self.diagnostics_list = []
        if rospy.get_param("~speak_warn", True):
            self.diagnostics_list.append(DiagnosticStatus.WARN)
        if rospy.get_param("~speak_error", True):
            self.diagnostics_list.append(DiagnosticStatus.ERROR)
        if rospy.get_param("~speak_stale", True):
            self.diagnostics_list.append(DiagnosticStatus.STALE)

        blacklist = rospy.get_param("~blacklist", [])
        self.speak_thread = SpeakThread(speak_rate, wait_speak, blacklist,
                                        language, volume, speak_interval)

        # run-stop
        self.run_stop = False

        # diag
        self.sub_diag = rospy.Subscriber(
            "/diagnostics_agg", DiagnosticArray,
            self.diag_cb, queue_size=1)
        self.speak_thread.start()

    def on_shutdown(self):
        self.speak_thread.stop()
        self.speak_thread.join()

    def diag_cb(self, msg):
        if self.run_stop:
            return
        target_status_list = filter(lambda n: n.level in self.diagnostics_list,
                                    msg.status)
        self.speak_thread.add(target_status_list)


if __name__ == '__main__':
    rospy.init_node("audible_warning")
    aw = AudibleWarning()  # NOQA
    rospy.on_shutdown(aw.on_shutdown)
    rospy.spin()
