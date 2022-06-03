#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import defaultdict
import heapq
from importlib import import_module
import re
from threading import Event
from threading import Lock
from threading import Thread

import actionlib
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from dynamic_reconfigure.server import Server
import rospy
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction
from sound_play.msg import SoundRequestGoal

from jsk_tools.cfg import AudibleWarningConfig as Config
from jsk_tools.diagnostics_utils import diagnostics_level_to_str
from jsk_tools.diagnostics_utils import filter_diagnostics_status_list
from jsk_tools.diagnostics_utils import is_leaf
from jsk_tools.inflection_utils import camel_to_snake


def expr_eval(expr):
    def eval_fn(topic, m, t):
        return eval(expr)
    return eval_fn


class SpeakThread(Thread):

    def __init__(self, rate=1.0, wait=True,
                 language='',
                 volume=1.0,
                 speak_interval=0,
                 wait_speak_duration_time=10,
                 diagnostics_level_list=None):
        super(SpeakThread, self).__init__()
        self.wait_speak_duration_time = wait_speak_duration_time
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.volume = volume
        self.lock = Lock()
        self.status_list = []
        self.speak_interval = speak_interval
        self.diagnostics_level_list = diagnostics_level_list or []
        tm = rospy.Time.now().to_sec() \
            - speak_interval
        self.previous_spoken_time = defaultdict(lambda tm=tm: tm)
        self.speak_flag = True
        self.language = language

        self.talk = actionlib.SimpleActionClient(
            "/robotsound", SoundRequestAction)
        self.talk.wait_for_server()

    def stop(self):
        self.event.set()

    def set_diagnostics_level_list(self, level_list):
        self.diagnostics_level_list = level_list

    def set_speak_flag(self, flag):
        if flag is True and self.speak_flag is False:
            # clear queue before start speaking.
            with self.lock:
                self.status_list = []
        self.speak_flag = flag
        if self.speak_flag is True:
            rospy.loginfo('audible warning is enabled. speak [{}] levels'
                          .format(', '.join(map(diagnostics_level_to_str,
                                                self.diagnostics_level_list))))
        else:
            rospy.loginfo('audible warning is disabled.')

    def set_volume(self, volume):
        volume = min(max(0.0, volume), 1.0)
        if self.volume != volume:
            self.volume = volume
            rospy.loginfo("audible warning's volume was set to {}".format(
                self.volume))

    def set_speak_interval(self, interval):
        interval = max(0.0, interval)
        if self.speak_interval != interval:
            self.speak_interval = interval
            rospy.loginfo("audible warning's speak interval was set to {}"
                          .format(self.speak_interval))

    def add(self, status_list):
        with self.lock:
            for status in status_list:
                heapq.heappush(
                    self.status_list,
                    (rospy.Time.now().to_sec(), status))

    def pop(self):
        with self.lock:
            while len(self.status_list) > 0:
                _, status = heapq.heappop(self.status_list)
                if is_leaf(status.name) is False:
                    continue
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
                if self.speak_flag is False:
                    continue
                if e.level not in self.diagnostics_level_list:
                    continue

                if e.level == DiagnosticStatus.OK:
                    prefix = 'ok.'
                elif e.level == DiagnosticStatus.WARN:
                    prefix = 'warning.'
                elif e.level == DiagnosticStatus.ERROR:
                    prefix = 'error.'
                elif e.level == DiagnosticStatus.STALE:
                    prefix = 'stale.'
                else:
                    prefix = 'ok.'
                sentence = prefix + e.name + ' ' + e.message
                sentence = camel_to_snake(sentence)
                sentence = sentence.replace('/', ' ')
                sentence = sentence.replace('_', ' ')
                rospy.loginfo('audible warning error name "{}"'.format(e.name))
                rospy.loginfo("audible warning talking: %s" % sentence)

                goal = SoundRequestGoal()
                goal.sound_request.sound = SoundRequest.SAY
                goal.sound_request.command = SoundRequest.PLAY_ONCE
                goal.sound_request.arg = sentence
                goal.sound_request.arg2 = self.language
                if hasattr(goal.sound_request, 'volume'):
                    goal.sound_request.volume = self.volume

                self.previous_spoken_time[e.name] = rospy.Time.now().to_sec()
                self.talk.send_goal(goal)
                if self.wait:
                    self.talk.wait_for_result(
                        rospy.Duration(self.wait_speak_duration_time))


class AudibleWarning(object):

    def __init__(self):
        speak_rate = rospy.get_param("~speak_rate", 1.0)
        wait_speak = rospy.get_param("~wait_speak", True)
        language = rospy.get_param('~language', '')
        seconds_to_start_speaking = rospy.get_param(
            '~seconds_to_start_speaking', 0)

        # Wait until seconds_to_start_speaking the time has passed.
        self.run_stop_enabled_time = None
        self.run_stop_disabled_time = None
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() \
                and (rospy.Time.now() - start_time).to_sec() \
                < seconds_to_start_speaking:
            rate.sleep()

        blacklist = rospy.get_param("~blacklist", [])
        self.blacklist_names = []
        self.blacklist_messages = []
        for bl in blacklist:
            if 'name' not in bl:
                name = re.compile(r'.')
            else:
                name = re.compile(bl['name'])
            self.blacklist_names.append(name)
            if 'message' not in bl:
                message = re.compile(r'.')
            else:
                message = re.compile(bl['message'])
            self.blacklist_messages.append(message)
        self.speak_thread = SpeakThread(
            speak_rate, wait_speak,
            language,
            wait_speak_duration_time=seconds_to_start_speaking)
        self.srv = Server(Config, self.config_callback)

        # run-stop
        self.speak_when_runstopped = rospy.get_param(
            '~speak_when_runstopped', True)
        self.run_stop = False
        self.run_stop_topic = rospy.get_param('~run_stop_topic', None)
        if self.run_stop_topic:
            run_stop_condition = rospy.get_param(
                '~run_stop_condition', 'm.data == True')
            run_stop_blacklist = rospy.get_param(
                '~run_stop_blacklist', [])
            self.run_stop_blacklist_names = []
            self.run_stop_blacklist_messages = []
            for bl in run_stop_blacklist:
                if 'name' not in bl:
                    name = re.compile(r'.')
                else:
                    name = re.compile(bl['name'])
                self.run_stop_blacklist_names.append(name)
                if 'message' not in bl:
                    message = re.compile(r'.')
                else:
                    message = re.compile(bl['message'])
                self.run_stop_blacklist_messages.append(message)
            self.run_stop_condition = expr_eval(run_stop_condition)
            self.run_stop_sub = rospy.Subscriber(
                self.run_stop_topic,
                rospy.AnyMsg,

                callback=self.run_stop_callback,
                queue_size=1)

        # diag
        self.sub_diag = rospy.Subscriber(
            "/diagnostics_agg", DiagnosticArray,
            self.diag_cb, queue_size=1)
        self.speak_thread.start()

    def config_callback(self, config, level):
        level_list = []
        if config.speak_ok:
            level_list.append(DiagnosticStatus.OK)
        if config.speak_warn:
            level_list.append(DiagnosticStatus.WARN)
        if config.speak_error:
            level_list.append(DiagnosticStatus.ERROR)
        if config.speak_stale:
            level_list.append(DiagnosticStatus.STALE)
        self.speak_thread.set_diagnostics_level_list(level_list)
        self.speak_thread.set_speak_flag(config.enable)
        self.speak_thread.set_volume(config.volume)
        self.speak_thread.set_speak_interval(config.speak_interval)
        self.ignore_time_after_runstop_is_enabled = \
            config.ignore_time_after_runstop_is_enabled
        self.ignore_time_after_runstop_is_disabled = \
            config.ignore_time_after_runstop_is_disabled
        return config

    def run_stop_callback(self, msg):
        if isinstance(msg, rospy.msg.AnyMsg):
            package, msg_type = msg._connection_header['type'].split('/')
            ros_pkg = package + '.msg'
            msg_class = getattr(import_module(ros_pkg), msg_type)
            self.run_stop_sub.unregister()
            deserialized_sub = rospy.Subscriber(
                self.run_stop_topic, msg_class, self.run_stop_callback)
            self.run_stop_sub = deserialized_sub
            return
        tm = rospy.Time.now()
        run_stop = self.run_stop_condition(
            self.run_stop_topic, msg, tm)
        if run_stop != self.run_stop:
            if run_stop is True:
                self.run_stop_enabled_time = tm
            else:
                self.run_stop_disabled_time = tm
        self.run_stop = run_stop

    def on_shutdown(self):
        self.speak_thread.stop()
        self.speak_thread.join()

    def diag_cb(self, msg):
        target_status_list = msg.status

        if self.ignore_time_after_runstop_is_enabled > 0.0:
            if self.run_stop_enabled_time is not None \
                    and ((rospy.Time.now()
                          - self.run_stop_enabled_time).to_sec <
                         self.ignore_time_after_runstop_is_enabled):
                return
        if self.ignore_time_after_runstop_is_disabled > 0.0:
            if self.run_stop_disabled_time is not None \
                    and ((rospy.Time.now()
                          - self.run_stop_disabled_time).to_sec <
                         self.ignore_time_after_runstop_is_disabled):
                return

        if self.run_stop:
            if self.speak_when_runstopped is False:
                rospy.logdebug('RUN STOP is pressed. Do not speak warning.')
                return

            target_status_list = filter_diagnostics_status_list(
                target_status_list,
                self.run_stop_blacklist_names,
                self.run_stop_blacklist_messages)

        target_status_list = filter_diagnostics_status_list(
            target_status_list, self.blacklist_names, self.blacklist_messages)
        self.speak_thread.add(target_status_list)


if __name__ == '__main__':
    rospy.init_node("audible_warning")
    aw = AudibleWarning()  # NOQA
    rospy.on_shutdown(aw.on_shutdown)
    rospy.spin()
