#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ConnectionBasedTransport of ROS 2 (rclpy).

The same interface as the ROS 1 one (transport.py), as a lifecycle node
like ConnectionBasedLifecycleNode (connection_based_lifecycle_node.cpp):
it is configured when created, and activated (its publishers enabled) with
subscribe() when an advertised topic gets its first subscriber, then
deactivated with unsubscribe() when none of them has any, which the
matched events of the publishers tell. ~always_subscribe and
~verbose_connection are supported; the vital check (~enable_vital_check)
and --inout are of ROS 1 only.
"""

import abc

from rclpy.event_handler import PublisherEventCallbacks
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
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
        'ConnectionBasedTransportBase', (LifecycleNode,), {})):

    def __init__(self, node_name, **kwargs):
        super(ConnectionBasedTransport, self).__init__(node_name, **kwargs)
        # the node of jsk_ros1_ros2_compat.rospy_rclpy_compat (None on ROS 1)
        self.node = self
        self.is_initialized = False
        # not _publishers, which is rclpy.node.Node's
        self._advertised_publishers = []
        self._ever_subscribed = False
        self._connection_status = NOT_SUBSCRIBED
        self._active = False
        self._always_subscribe = ros.get_param(
            self, '~always_subscribe', False)
        self._verbose_connection = ros.get_param(
            self, '~verbose_connection', False)
        self._warn_timer = self.create_timer(
            5.0, self._warn_never_subscribed_cb)

    def _post_init(self):
        self.is_initialized = True
        if not self._advertised_publishers:
            raise RuntimeError(
                'No publishers registered.'
                ' Have you called ConnectionBasedTransport.advertise?')
        self.trigger_configure()
        if self._always_subscribe:
            self.trigger_activate()
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
                self,
                '[{name}] subscribes topics only with'
                " child subscribers. Set '~always_subscribe' as True"
                ' to have it subscribe always.'.format(name=self.get_name()))

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
        if not self.is_initialized or self._always_subscribe:
            return
        if any(pub.get_subscription_count() > 0
               for pub in self._advertised_publishers):
            if self._connection_status == NOT_SUBSCRIBED:
                if self._verbose_connection:
                    ros.loginfo(self, 'Subscribe input topics.')
                if not self._active:
                    self.trigger_activate()
                self.subscribe()
                self._connection_status = SUBSCRIBED
                self._ever_subscribed = True
        elif self._connection_status == SUBSCRIBED:
            if self._verbose_connection:
                ros.loginfo(self, 'Unsubscribe input topics.')
            self.unsubscribe()
            self._connection_status = NOT_SUBSCRIBED
            if self._active:
                self.trigger_deactivate()

    def _matched_cb(self, info):
        if self._verbose_connection:
            ros.loginfo(self, 'New connection or disconnection detected.')
        self._update_connection()

    def _unsubscribe_if_subscribed(self):
        if self._connection_status == SUBSCRIBED:
            self.unsubscribe()
            self._connection_status = NOT_SUBSCRIBED

    def is_active(self):
        """True while the lifecycle node is active."""
        return self._active

    def on_activate(self, state):
        ret = super(ConnectionBasedTransport, self).on_activate(state)
        self._active = ret == TransitionCallbackReturn.SUCCESS
        return ret

    def on_deactivate(self, state):
        ret = super(ConnectionBasedTransport, self).on_deactivate(state)
        self._active = ret != TransitionCallbackReturn.SUCCESS
        return ret

    def on_cleanup(self, state):
        self._unsubscribe_if_subscribed()
        return super(ConnectionBasedTransport, self).on_cleanup(state)

    def on_shutdown(self, state):
        self._unsubscribe_if_subscribed()
        self._active = False
        return super(ConnectionBasedTransport, self).on_shutdown(state)

    def advertise(self, topic, msg_type, queue_size=1, latch=False):
        """A lifecycle publisher, enabled while subscribed."""
        if latch:
            qos = QoSProfile(depth=queue_size,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        else:
            qos = queue_size
        pub = self.create_lifecycle_publisher(
            msg_type, ros.private_name(topic), qos,
            event_callbacks=PublisherEventCallbacks(matched=self._matched_cb))
        self._advertised_publishers.append(pub)
        return pub
