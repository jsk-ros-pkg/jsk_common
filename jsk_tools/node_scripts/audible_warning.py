#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import defaultdict
import heapq
from importlib import import_module
import re
from threading import Event
from threading import Lock
from threading import Thread

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import std_msgs.msg

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
from jsk_tools.diagnostics_utils import diagnostics_level_to_str
from jsk_tools.diagnostics_utils import filter_diagnostics_status_list
from jsk_tools.diagnostics_utils import is_leaf
from jsk_tools.inflection_utils import camel_to_snake
from jsk_tools.string_utils import multiple_whitespace_to_one

if ROS_VERSION == 2:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import SetParametersResult
else:
    import actionlib
    from dynamic_reconfigure.server import Server
    import rospy
    from sound_play.msg import SoundRequest
    from sound_play.msg import SoundRequestAction
    from sound_play.msg import SoundRequestGoal

    from jsk_tools.cfg import AudibleWarningConfig as Config


def expr_eval(expr):
    def eval_fn(topic, m, t):
        return eval(expr)
    return eval_fn


class SpeakThread(Thread):

    def __init__(self, node, rate=1.0, wait=True,
                 language='',
                 volume=1.0,
                 speak_interval=0,
                 wait_speak_duration_time=10,
                 diagnostics_level_list=None):
        super(SpeakThread, self).__init__()
        self.node = node
        self.wait_speak_duration_time = wait_speak_duration_time
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.volume = volume
        self.lock = Lock()
        self.status_list = []
        self.speak_interval = speak_interval
        self.diagnostics_level_list = diagnostics_level_list or []
        tm = ros_compat.now_sec(node) \
            - speak_interval
        self.previous_spoken_time = defaultdict(lambda tm=tm: tm)
        self.speak_flag = True
        self.language = language

        self.pub_original_text = ros_compat.create_publisher(
            node, '~output/original_text',
            std_msgs.msg.String,
            queue_size=1)
        self.pub_speak_text = ros_compat.create_publisher(
            node, '~output/text',
            std_msgs.msg.String,
            queue_size=1)

        if ROS_VERSION == 1:
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
            ros_compat.loginfo(
                self.node,
                'audible warning is enabled. speak [{}] levels'.format(
                    ', '.join(map(diagnostics_level_to_str,
                                  self.diagnostics_level_list))))
        else:
            ros_compat.loginfo(self.node, 'audible warning is disabled.')

    def set_volume(self, volume):
        volume = min(max(0.0, volume), 1.0)
        if self.volume != volume:
            self.volume = volume
            ros_compat.loginfo(
                self.node,
                "audible warning's volume was set to {}".format(self.volume))

    def set_speak_interval(self, interval):
        interval = max(0.0, interval)
        if self.speak_interval != interval:
            self.speak_interval = interval
            ros_compat.loginfo(
                self.node,
                "audible warning's speak interval was set to {}".format(
                    self.speak_interval))

    def add(self, status_list):
        with self.lock:
            for status in status_list:
                if is_leaf(status.name) is False:
                    continue
                if ros_compat.now_sec(self.node) \
                        - self.previous_spoken_time[status.name] \
                        < self.speak_interval:
                    continue
                heapq.heappush(
                    self.status_list,
                    (ros_compat.now_sec(self.node), status))

    def pop(self):
        with self.lock:
            while len(self.status_list) > 0:
                _, status = heapq.heappop(self.status_list)
                if is_leaf(status.name) is False:
                    continue
                if ros_compat.now_sec(self.node) \
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
                sentence = sentence.replace(':', ' colon ')
                sentence = multiple_whitespace_to_one(sentence)
                ros_compat.loginfo(
                    self.node, 'audible warning error name "{}"'.format(e.name))
                ros_compat.loginfo(
                    self.node, "audible warning talking: %s" % sentence)
                self.pub_original_text.publish(
                    std_msgs.msg.String(data=prefix + ' ' + e.name + ' ' + e.message)
                    if ROS_VERSION == 2 else prefix + ' ' + e.name + ' ' + e.message)
                self.pub_speak_text.publish(
                    std_msgs.msg.String(data="audible warning talking: %s" % sentence)
                    if ROS_VERSION == 2 else "audible warning talking: %s" % sentence)

                self.previous_spoken_time[e.name] = ros_compat.now_sec(self.node)

                if ROS_VERSION == 2:
                    # [TTS disabled under ROS2] there is no ROS2/ament port
                    # of the `sound_play` package (confirmed: no
                    # ros-jazzy-sound-play, only the unrelated
                    # ros-one-sound-play), so /robotsound has no ROS2
                    # target to send an action goal to.
                    ros_compat.loginfo(
                        self.node, '[TTS disabled under ROS2] would speak: %s' % sentence)
                    continue

                goal = SoundRequestGoal()
                goal.sound_request.sound = SoundRequest.SAY
                goal.sound_request.command = SoundRequest.PLAY_ONCE
                goal.sound_request.arg = sentence
                goal.sound_request.arg2 = self.language
                if hasattr(goal.sound_request, 'volume'):
                    goal.sound_request.volume = self.volume

                self.talk.send_goal(goal)
                if self.wait:
                    self.talk.wait_for_result(
                        rospy.Duration(self.wait_speak_duration_time))


class AudibleWarning(object):

    def __init__(self, node=None):
        self.node = node
        speak_rate = ros_compat.get_param(node, "~speak_rate", 1.0 / 100.0)
        wait_speak = ros_compat.get_param(node, "~wait_speak", True)
        language = ros_compat.get_param(node, '~language', '')
        seconds_to_start_speaking = ros_compat.get_param(
            node, '~seconds_to_start_speaking', 0)
        wait_speak_duration_time = ros_compat.get_param(
            node, '~wait_speak_duration_time', 30.0)

        # Wait until seconds_to_start_speaking the time has passed.
        self.run_stop_enabled_time = None
        self.run_stop_disabled_time = None
        rate = ros_compat.Rate(node, 10)
        start_time = ros_compat.now_sec(node)
        while not ros_compat.is_shutdown(node) \
                and ros_compat.now_sec(node) - start_time \
                < seconds_to_start_speaking:
            rate.sleep()

        # NOTE: ~blacklist/~run_stop_blacklist (lists of {name, message}
        # regex filters) are ROS1-only for now -- ROS2 parameters can't
        # hold list-of-dict structures the way ROS1 rosparam YAML could.
        self.blacklist_names = []
        self.blacklist_messages = []
        if ROS_VERSION == 1:
            blacklist = ros_compat.get_param(node, "~blacklist", [])
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
            node, speak_rate, wait_speak,
            language,
            wait_speak_duration_time=wait_speak_duration_time)

        if ROS_VERSION == 2:
            self._declare_ros2_parameters()
            node.add_on_set_parameters_callback(self._on_set_parameters_ros2)
            self._apply_ros2_parameters()
        else:
            self.srv = Server(Config, self.config_callback)

        # run-stop
        self.run_stop = False
        self.run_stop_blacklist_names = []
        self.run_stop_blacklist_messages = []
        run_stop_topic = ros_compat.get_param(node, '~run_stop_topic', None)
        if run_stop_topic:
            run_stop_condition = ros_compat.get_param(
                node, '~run_stop_condition', 'm.data == True')
            self.run_stop_condition = expr_eval(run_stop_condition)
            if ROS_VERSION == 2:
                # NOTE: ROS1's rospy.AnyMsg + connection-header
                # introspection (subscribing without knowing the message
                # type ahead of time) has no rclpy equivalent; always
                # subscribe as std_msgs/Bool under ROS2.
                self.run_stop_sub = ros_compat.create_subscription(
                    node, run_stop_topic, std_msgs.msg.Bool,
                    self.run_stop_callback, 1)
            else:
                run_stop_blacklist = ros_compat.get_param(
                    node, '~run_stop_blacklist', [])
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
                self.run_stop_sub = ros_compat.create_subscription(
                    node, run_stop_topic, rospy.AnyMsg,
                    self.run_stop_callback, 1)

        # diag
        self.sub_diag = ros_compat.create_subscription(
            node, "/diagnostics_agg", DiagnosticArray, self.diag_cb, 1)
        self.speak_thread.start()

    def _declare_ros2_parameters(self):
        node = self.node
        node.declare_parameter('enable', True)
        node.declare_parameter('speak_ok', False)
        node.declare_parameter('speak_stale', True)
        node.declare_parameter('speak_warn', True)
        node.declare_parameter('speak_error', True)
        node.declare_parameter('speak_when_runstopped', True)
        node.declare_parameter('volume', 1.0)
        node.declare_parameter('speak_interval', 120.0)
        node.declare_parameter('ignore_time_after_runstop_is_enabled', 0.0)
        node.declare_parameter('ignore_time_after_runstop_is_disabled', 0.0)

    def _apply_ros2_parameters(self):
        node = self.node
        level_list = []
        if node.get_parameter('speak_ok').value:
            level_list.append(DiagnosticStatus.OK)
        if node.get_parameter('speak_warn').value:
            level_list.append(DiagnosticStatus.WARN)
        if node.get_parameter('speak_error').value:
            level_list.append(DiagnosticStatus.ERROR)
        if node.get_parameter('speak_stale').value:
            level_list.append(DiagnosticStatus.STALE)
        self.speak_thread.set_diagnostics_level_list(level_list)
        self.speak_thread.set_speak_flag(node.get_parameter('enable').value)
        self.speak_thread.set_volume(node.get_parameter('volume').value)
        self.speak_thread.set_speak_interval(
            node.get_parameter('speak_interval').value)
        self.ignore_time_after_runstop_is_enabled = node.get_parameter(
            'ignore_time_after_runstop_is_enabled').value
        self.ignore_time_after_runstop_is_disabled = node.get_parameter(
            'ignore_time_after_runstop_is_disabled').value
        self.speak_when_runstopped = node.get_parameter(
            'speak_when_runstopped').value

    def _on_set_parameters_ros2(self, params):
        # values in `params` are the pending new values; self.node's own
        # parameter storage is not updated until after this callback
        # returns successfully, so apply from `params` where present.
        overrides = {p.name: p.value for p in params}

        def value(name):
            return overrides.get(name, self.node.get_parameter(name).value)

        level_list = []
        if value('speak_ok'):
            level_list.append(DiagnosticStatus.OK)
        if value('speak_warn'):
            level_list.append(DiagnosticStatus.WARN)
        if value('speak_error'):
            level_list.append(DiagnosticStatus.ERROR)
        if value('speak_stale'):
            level_list.append(DiagnosticStatus.STALE)
        self.speak_thread.set_diagnostics_level_list(level_list)
        self.speak_thread.set_speak_flag(value('enable'))
        self.speak_thread.set_volume(value('volume'))
        self.speak_thread.set_speak_interval(value('speak_interval'))
        if 'ignore_time_after_runstop_is_enabled' in overrides:
            self.ignore_time_after_runstop_is_enabled = \
                overrides['ignore_time_after_runstop_is_enabled']
        if 'ignore_time_after_runstop_is_disabled' in overrides:
            self.ignore_time_after_runstop_is_disabled = \
                overrides['ignore_time_after_runstop_is_disabled']
        if 'speak_when_runstopped' in overrides:
            self.speak_when_runstopped = overrides['speak_when_runstopped']
        return SetParametersResult(successful=True)

    def config_callback(self, config, level):
        # ROS1 dynamic_reconfigure callback.
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
        self.speak_when_runstopped = config.speak_when_runstopped
        return config

    def run_stop_callback(self, msg):
        if ROS_VERSION == 1 and isinstance(msg, rospy.msg.AnyMsg):
            package, msg_type = msg._connection_header['type'].split('/')
            ros_pkg = package + '.msg'
            msg_class = getattr(import_module(ros_pkg), msg_type)
            self.run_stop_sub.unregister()
            self.run_stop_sub = ros_compat.create_subscription(
                self.node, self.run_stop_topic, msg_class, self.run_stop_callback)
            msg = msg_class().deserialize(msg._buff)
        tm = ros_compat.now_sec(self.node)
        run_stop = self.run_stop_condition(
            'run_stop', msg, tm)
        if run_stop != self.run_stop:
            if run_stop is True:
                self.run_stop_enabled_time = tm
                ros_compat.loginfo(self.node, 'Audible Warning: Runstop is enabled.')
            else:
                self.run_stop_disabled_time = tm
                ros_compat.loginfo(self.node, 'Audible Warning: Runstop is disabled.')
        self.run_stop = run_stop

    def on_shutdown(self):
        self.speak_thread.stop()
        self.speak_thread.join()

    def diag_cb(self, msg):
        target_status_list = msg.status
        now = ros_compat.now_sec(self.node)

        if self.ignore_time_after_runstop_is_enabled > 0.0:
            if self.run_stop_enabled_time is not None \
                    and (now - self.run_stop_enabled_time <
                         self.ignore_time_after_runstop_is_enabled):
                return
        if self.ignore_time_after_runstop_is_disabled > 0.0:
            if self.run_stop_disabled_time is not None \
                    and (now - self.run_stop_disabled_time <
                         self.ignore_time_after_runstop_is_disabled):
                return

        if self.run_stop:
            if self.speak_when_runstopped is False:
                ros_compat.logdebug(
                    self.node, 'RUN STOP is pressed. Do not speak warning.')
                return

            target_status_list = filter_diagnostics_status_list(
                target_status_list,
                self.run_stop_blacklist_names,
                self.run_stop_blacklist_messages)

        target_status_list = filter_diagnostics_status_list(
            target_status_list, self.blacklist_names, self.blacklist_messages)
        self.speak_thread.add(target_status_list)


def main():
    if ROS_VERSION == 2:
        rclpy.init()
        node = Node('audible_warning')
        aw = AudibleWarning(node)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            aw.on_shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        rospy.init_node("audible_warning")
        aw = AudibleWarning()  # NOQA
        rospy.on_shutdown(aw.on_shutdown)
        rospy.spin()


if __name__ == '__main__':
    main()
