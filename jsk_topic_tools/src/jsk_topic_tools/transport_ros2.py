#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ConnectionBasedTransport of ROS 2 (rclpy).

The same interface as the ROS 1 one (transport.py), with the rclpy node
given to the constructor: subscribe() is called when an advertised topic
gets its first subscriber and unsubscribe() when none of them has any,
which the matched events of the publishers tell. ~always_subscribe is
supported; the vital check (~enable_vital_check) and --inout are of
ROS 1 only.
"""

import abc

from rclpy.event_handler import PublisherEventCallbacks
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


__all__ = ('ConnectionBasedTransport',)


SUBSCRIBED = 0
NOT_SUBSCRIBED = 1


class MetaConnectionBasedTransport(abc.ABCMeta):
    def __call__(cls, *args, **kwargs):
        """Called when you call ConnectionBasedTransport()"""
        obj = type.__call__(cls, *args, **kwargs)
        obj._post_init()
        return obj


# the metaclass in a way python2 can byte-compile too (catkin installs it)
class ConnectionBasedTransport(MetaConnectionBasedTransport(
        'ConnectionBasedTransportBase', (object,), {})):

    def __init__(self, node):
        self.node = node
        self.is_initialized = False
        self._publishers = []
        self._ever_subscribed = False
        self._connection_status = NOT_SUBSCRIBED
        self._warn_timer = self.node.create_timer(
            5.0, self._warn_never_subscribed_cb)

    def _post_init(self):
        self.is_initialized = True
        if not self._publishers:
            raise RuntimeError(
                'No publishers registered.'
                ' Have you called ConnectionBasedTransport.advertise?')
        if ros.get_param(self.node, '~always_subscribe', False):
            self.subscribe()
            self._connection_status = SUBSCRIBED
            self._ever_subscribed = True
        else:
            # subscribers matched before the publishers were advertised
            self._update_connection()

    def _warn_never_subscribed_cb(self):
        self._warn_timer.cancel()
        if not self._ever_subscribed:
            ros.logwarn(
                self.node,
                '[{name}] subscribes topics only with'
                " child subscribers. Set '~always_subscribe' as True"
                ' to have it subscribe always.'.format(
                    name=ros.get_name(self.node)))

    @abc.abstractmethod
    def subscribe(self):
        raise NotImplementedError

    @abc.abstractmethod
    def unsubscribe(self):
        raise NotImplementedError

    def is_subscribed(self):
        return self._connection_status == SUBSCRIBED

    def poke(self):
        """Nothing to do on ROS 2, which has no vital check."""

    def _update_connection(self):
        if not self.is_initialized:
            return  # _post_init() does it
        if any(pub.get_subscription_count() > 0
               for pub in self._publishers):
            if self._connection_status == NOT_SUBSCRIBED:
                self.subscribe()
                self._connection_status = SUBSCRIBED
                self._ever_subscribed = True
        else:
            if ros.get_param(self.node, '~always_subscribe', False):
                return  # do not unsubscribe
            if self._connection_status == SUBSCRIBED:
                self.unsubscribe()
                self._connection_status = NOT_SUBSCRIBED

    def _matched_cb(self, info):
        ros.logdebug(self.node, 'subscribers of an advertised topic: {}'
                     .format(info.current_count))
        self._update_connection()

    def advertise(self, topic, msg_type, queue_size=1, latch=False):
        if latch:
            qos = QoSProfile(depth=queue_size,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        else:
            qos = queue_size
        pub = self.node.create_publisher(
            msg_type, ros.private_name(topic), qos,
            event_callbacks=PublisherEventCallbacks(matched=self._matched_cb))
        self._publishers.append(pub)
        return pub
