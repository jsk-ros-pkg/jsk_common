#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum

import rospy


ConnectionStatus = Enum('ConnectionStatus', 'SUBSCRIBED NOT_SUBSCRIBED')


class ConnectionBasedTransport(rospy.SubscribeListener):
    def __init__(self):
        super(ConnectionBasedTransport, self).__init__()
        self._publishers = []
        self._ever_subscribed = False
        self._connection_status = ConnectionStatus.NOT_SUBSCRIBED
        rospy.Timer(rospy.Duration(5),
                    self._warn_never_subscribed_cb, oneshot=True)

    def _warn_never_subscribed_cb(self, timer_event):
        if not self._ever_subscribed:
            rospy.logwarn('[{name}] subscribes topics only with'
                          ' child subscribers.'.format(name=rospy.get_name()))

    def subscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def unsubscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def peer_subscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is subscribed'.format(topic=args[0]))
        if self._connection_status == ConnectionStatus.NOT_SUBSCRIBED:
            self.subscribe()
            if not self._ever_subscribed:
                self._ever_subscribed = True

    def peer_unsubscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is unsubscribed'.format(topic=args[0]))
        for pub in self._publishers:
            if pub.get_num_connections() == 0:
                self.unsubscribe()

    def advertise(self, *args, **kwargs):
        # subscriber_listener should be 'self'
        # to detect connection and disconnection of the publishing topics
        assert len(args) < 3 or args[2] is None
        assert kwargs.get('subscriber_listener') is None
        kwargs['subscriber_listener'] = self

        pub = rospy.Publisher(*args, **kwargs)
        self._publishers.append(pub)
        return pub
