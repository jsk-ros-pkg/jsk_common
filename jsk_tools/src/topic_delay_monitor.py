#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import math

import rosgraph
import rospy
import rostopic

import numpy as np
from texttable import Texttable


class ROSTopicDelay(object):

    def __init__(self, window_size):
        import threading
        self.lock = threading.Lock()
        self.last_msg_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.delays = []

        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def callback_delay(self, msg):
        if msg._has_header is False:
            rospy.logerr('msg does not have header')
            return
        with self.lock:
            curr_rostime = rospy.get_rostime()

            # time reset
            if curr_rostime.is_zero():
                if len(self.delays) > 0:
                    print("time has reset, resetting counters")
                    self.delays = []
                return

            curr = curr_rostime.to_sec()
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.delays = []
            else:
                self.delays.append(curr_rostime.to_time()
                                   -  msg.header.stamp.to_time())
                self.msg_tn = curr

            if len(self.delays) > self.window_size - 1:
                self.delays.pop(0)

    def get_delay(self):
        if self.msg_tn == self.last_msg_tn:
            return
        with self.lock:
            if not self.delays:
                return
            n = len(self.delays)

            mean = sum(self.delays) / n
            rate = 1. / mean if mean > 0 else 0

            std_dev = math.sqrt(sum((x - mean)**2 for x in self.delays) / n)

            max_delta = max(self.delays)
            min_delta = min(self.delays)

            self.last_msg_tn = self.msg_tn
        return mean, min_delta, max_delta, std_dev, n + 1


def get_parent_topics(topic, parent_topics=None, impl=False):
    master = rosgraph.Master(ID)
    publications, subscriptions, _ = master.getSystemState()
    if parent_topics is None:
        parent_topics = []
    pub_nodes = [l for t, l in publications if t == topic]
    if not pub_nodes:
        return parent_topics
    for node in pub_nodes[0]:
        subs = [t for t, l in subscriptions if node in l]
        subs = filter(lambda x: x not in parent_topics, subs)  # get unique
        parent_topics.extend(subs)
        if not impl:
            return parent_topics
        for sub in subs:
            parent_topics = get_parent_topics(sub, parent_topics, impl=impl)
    return parent_topics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topics', nargs='+',
                        help='leaf topic names of topic graph')
    parser.add_argument('-p', '--search-parent', action='store_true',
                        help='search parent topics to check')
    args = parser.parse_args()
    topics = args.topics
    if args.search_parent:
        # get all parent topics
        parent_topics = []
        for topic in topics:
            parent_topics.extend(get_parent_topics(topic, impl=True))
        parent_topics = list(set(parent_topics))
        topics.extend(parent_topics)
    topics = np.array(topics)
    # prepare topic hz checkers
    delay_checkers = []
    for t in topics:
        rt = ROSTopicDelay(window_size=-1)
        delay_checkers.append(rt)
        msg_class, real_topic, _ = rostopic.get_topic_class(t, blocking=True)
        rospy.Subscriber(real_topic, msg_class, rt.callback_delay)
    rospy.loginfo('subscribed {} topics'.format(len(topics)))
    # main loop
    while not rospy.is_shutdown():
        # wait for first message
        if not all(rt.delays for rt in delay_checkers):
            continue
        # collect topic hz stats
        show_topics, stats = [], []
        for i, rt in enumerate(delay_checkers):
            delay_stat = rt.get_delay()
            if delay_stat is None:
                continue
            delay, min_delta, max_delta, std_dev, window = delay_stat
            show_topics.append(topics[i])
            stats.append([delay, min_delta, max_delta, std_dev, window])
        show_topics, stats = np.array(show_topics), np.array(stats)
        if stats.size == 0:
            rospy.logwarn('no messages')
        else:
            sort_indices = np.argsort(stats[:, 0])[::-1]
            show_topics, stats = show_topics[sort_indices], stats[sort_indices]
            stats = np.hstack((show_topics.reshape(-1, 1), stats)).tolist()
            # print stats result
            header = ['topic', 'delay', 'min_delta',
                      'max_delta', 'std_dev', 'window']
            table = Texttable(max_width=0)
            table.set_deco(Texttable.HEADER)
            table.add_rows([header] + stats)
            print(table.draw())
        # wait for next check
        rospy.rostime.wallsleep(1.0)


if __name__ == '__main__':
    NAME = 'topic_delay_monitor'
    ID = '/' + NAME
    rospy.init_node(NAME)
    main()
