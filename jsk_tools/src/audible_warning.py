#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import defaultdict
import actionlib
import rospy
from threading import Event
from threading import Thread
from threading import Lock
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus

cached_paths = {}
cached_result = {}


def is_leaf(original_path):
    if hasattr(cached_result, original_path):
        return cached_result[original_path]

    path = original_path[1:].split('/')

    def _is_leaf(cached, path):
        if not isinstance(path, list):
            raise ValueError
        if len(path) == 0:
            raise ValueError
        if len(path) == 1:
            if path[0] in cached:
                return False
            else:
                return True

        parent = path[0]
        child = path[1:]
        if parent not in cached:
            cached[parent] = {}
        return _is_leaf(cached[parent], child)

    result = _is_leaf(cached_paths, path)
    if result is False:
        cached_result[original_path] = result
    return result


class SpeakThread(Thread):

    def __init__(self, rate=1.0, wait=True, blacklist=None):
        super(SpeakThread, self).__init__()
        self.memo = {}
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.lock = Lock()
        self.stale = []
        self.error = []
        self.history = defaultdict(lambda: 10)
        self.blacklist = blacklist

        self.talk = actionlib.SimpleActionClient(
            "/robotsound", SoundRequestAction)
        self.talk.wait_for_server()

        self.language = 'en'

    def stop(self):
        self.event.set()

    def sort(self, error):
        ret = {}
        for s in error:
            ns = s.name.split()[0]
            if is_leaf(ns) is False:
                continue
            if any(filter(lambda n: n in ns, self.blacklist)):
                continue
            ret[ns] = s
        return ret.values()

    def add(self, stale, error):
        with self.lock:
            self.stale = self.sort(stale)
            self.error = self.sort(error)

    def pop(self):
        with self.lock:
            for e in self.stale + self.error:
                if e.name not in self.history.keys():
                    self.history[e.name] += 1
                    return e
            return None

    def sweep(self):
        for k in self.history.keys():
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
                    self.talk.wait_for_result(rospy.Duration(10.0))
            self.sweep()


class AudibleWarning(object):
    def __init__(self):
        self.history = []
        self.error = []
        self.stale = []

        speak_rate = rospy.get_param("~speak_rate", 1.0)
        speak_wait = rospy.get_param("~speak_wait", True)
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
        stale = filter(lambda s: s.level == DiagnosticStatus.STALE, msg.status)
        self.speak_thread.add(stale, error)


if __name__ == '__main__':
    rospy.init_node("audible_warning")
    aw = AudibleWarning()  # NOQA
    rospy.on_shutdown(aw.on_shutdown)
    rospy.spin()
