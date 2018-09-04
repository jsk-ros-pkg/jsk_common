#!/usr/bin/env python

import abc
import argparse
from distutils.version import LooseVersion
import pkg_resources
import sys

import rospy

from jsk_topic_tools.name_utils import unresolve_name


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


class ConnectionBasedTransport(rospy.SubscribeListener):

    __metaclass__ = MetaConnectionBasedTransport

    def __init__(self):
        super(ConnectionBasedTransport, self).__init__()
        self.is_initialized = False
        self._publishers = []
        self._ever_subscribed = False
        self._connection_status = NOT_SUBSCRIBED
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

    @abc.abstractmethod
    def subscribe(self):
        raise NotImplementedError

    @abc.abstractmethod
    def unsubscribe(self):
        raise NotImplementedError

    def is_subscribed(self):
        return self._connection_status == SUBSCRIBED

    def peer_subscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is subscribed'.format(topic=args[0]))
        if self._connection_status == NOT_SUBSCRIBED:
            self.subscribe()
            self._connection_status = SUBSCRIBED
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

        pub = rospy.Publisher(*args, **kwargs)
        self._publishers.append(pub)
        return pub
