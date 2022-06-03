#!/usr/bin/env python

import abc
import argparse
from distutils.version import LooseVersion
import sys

from diagnostic_msgs.msg import DiagnosticStatus
import pkg_resources
import rospy

from jsk_topic_tools.name_utils import unresolve_name
from jsk_topic_tools import TimeredDiagnosticUpdater


__all__ = ('ConnectionBasedTransport',)


SUBSCRIBED = 0
NOT_SUBSCRIBED = 1


# define a new metaclass which overrides the '__call__' function
# See: http://martyalchin.com/2008/jan/10/simple-plugin-framework/
class MetaConnectionBasedTransport(abc.ABCMeta):
    def __call__(cls, *args, **kwargs):
        """Called when you call ConnectionBasedTransport()"""
        obj = type.__call__(cls, *args, **kwargs)

        # display node input/output topics
        parser = argparse.ArgumentParser()
        parser.add_argument('--inout', action='store_true')
        args = parser.parse_args(rospy.myargv()[1:])
        if args.inout:
            obj.subscribe()
            tp_manager = rospy.topics.get_topic_manager()
            print('Publications:')
            for topic, topic_type in tp_manager.get_publications():
                if topic == '/rosout':
                    continue
                topic = unresolve_name(rospy.get_name(), topic)
                print(' * {0} [{1}]'.format(topic, topic_type))
            print('Subscriptions:')
            for topic, topic_type in tp_manager.get_subscriptions():
                topic = unresolve_name(rospy.get_name(), topic)
                print(' * {0} [{1}]'.format(topic, topic_type))
            sys.exit(0)

        obj._post_init()
        return obj


class _Publisher(rospy.Publisher):

    def __init__(self, *args, **kwargs):
        super(_Publisher, self).__init__(*args, **kwargs)
        self.last_published_time = rospy.Time.from_sec(0)

    def publish(self, *args, **kwargs):
        super(_Publisher, self).publish(*args, **kwargs)
        self.last_published_time = rospy.Time.now()


class ConnectionBasedTransport(rospy.SubscribeListener):

    __metaclass__ = MetaConnectionBasedTransport

    def __init__(self):
        super(ConnectionBasedTransport, self).__init__()
        self.is_initialized = False
        self._publishers = []
        self._ever_subscribed = False
        self._connection_status = NOT_SUBSCRIBED

        if rospy.get_param('~enable_vital_check', True):
            self.diagnostic_updater = TimeredDiagnosticUpdater(
                rospy.Duration(1.0))
            self.node_name = rospy.get_name()
            self.diagnostic_updater.set_hardware_id(self.node_name)
            self.diagnostic_updater.add(self.node_name,
                                        self.update_diagnostic)

            self.use_warn = rospy.get_param(
                '/diagnostic_nodelet/use_warn',
                rospy.get_param('~use_warn', False))
            if self.use_warn:
                self.diagnostic_error_level = DiagnosticStatus.WARN
            else:
                self.diagnostic_error_level = DiagnosticStatus.ERROR
            self.vital_rate = rospy.get_param('~vital_rate', 1.0)
            self.dead_duration = 1.0 / self.vital_rate
            self.diagnostic_updater.start()

        kwargs = dict(
            period=rospy.Duration(5),
            callback=self._warn_never_subscribed_cb,
            oneshot=True,
        )
        if (LooseVersion(pkg_resources.get_distribution('rospy').version) >=
                LooseVersion('1.12.0')):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            kwargs['reset'] = True
        rospy.Timer(**kwargs)

    def _post_init(self):
        self.is_initialized = True
        if not self._publishers:
            raise RuntimeError(
                'No publishers registered.'
                ' Have you called ConnectionBasedTransport.advertise?')
        if rospy.get_param('~always_subscribe', False):
            self.subscribe()
            self._connection_status = SUBSCRIBED
            self._ever_subscribed = True

    def _warn_never_subscribed_cb(self, timer_event):
        if not self._ever_subscribed:
            rospy.logwarn(
                '[{name}] subscribes topics only with'
                " child subscribers. Set '~always_subscribe' as True"
                ' to have it subscribe always.'.format(name=rospy.get_name()))

    def _is_running(self):
        current_time = rospy.Time.now()
        # check subscribed topics are published.
        return all([
            (current_time.to_sec() - pub.last_published_time.to_sec())
            < self.dead_duration
            for pub in self._publishers
            if pub.get_num_connections() > 0])

    @abc.abstractmethod
    def subscribe(self):
        raise NotImplementedError

    @abc.abstractmethod
    def unsubscribe(self):
        raise NotImplementedError

    def is_subscribed(self):
        return self._connection_status == SUBSCRIBED

    def poke(self):
        """Update the time of last_published_time.

        Update the time of last_published_time
        to make it possible to take the difference time
        between the time of start subscribing and the current time.
        """
        start_time = rospy.Time.now()
        for pub in self._publishers:
            pub.last_published_time = start_time

    def peer_subscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is subscribed'.format(topic=args[0]))
        if self._connection_status == NOT_SUBSCRIBED:
            self.subscribe()
            self._connection_status = SUBSCRIBED

            self.poke()

            if not self._ever_subscribed:
                self._ever_subscribed = True

    def peer_unsubscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is unsubscribed'.format(topic=args[0]))
        if rospy.get_param('~always_subscribe', False):
            return  # do not unsubscribe
        if self._connection_status == NOT_SUBSCRIBED:
            return  # no need to unsubscribe
        for pub in self._publishers:
            if pub.get_num_connections() > 0:
                break
        else:
            self.unsubscribe()
            self._connection_status = NOT_SUBSCRIBED

    def advertise(self, *args, **kwargs):
        # subscriber_listener should be 'self'
        # to detect connection and disconnection of the publishing topics
        assert len(args) < 3 or args[2] is None
        assert kwargs.get('subscriber_listener') is None
        kwargs['subscriber_listener'] = self

        pub = _Publisher(*args, **kwargs)
        self._publishers.append(pub)
        return pub

    def update_diagnostic(self, stat):
        if self._connection_status == SUBSCRIBED:
            if self._is_running():
                stat.summary(
                    DiagnosticStatus.OK,
                    '{} is running'.format(self.node_name))
            else:
                stat.summary(
                    self.diagnostic_error_level,
                    "{} is not running for {} sec".format(
                        self.node_name, self.dead_duration))
        else:
            stat.summary(
                DiagnosticStatus.OK,
                '{} is not subscribed'.format(self.node_name))
        topic_names = ', '.join([pub.name for pub in self._publishers])
        stat.add('watched topics', topic_names)
        for pub in self._publishers:
            stat.add(pub.name, '{} subscribers'.format(
                pub.get_num_connections()))
