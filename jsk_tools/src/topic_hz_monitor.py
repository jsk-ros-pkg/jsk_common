#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import math

import rosgraph
import rospy
from rostopic import ROSTopicHz as _ROSTopicHz

import numpy as np
from texttable import Texttable


class ROSTopicHz(_ROSTopicHz):

    def get_hz(self):
        """Get hz stats

        @returns: tuple of stat results
            rate, min_delta, max_delta, standard deviation, window number
        """
        if not self.times:
            return
        elif self.msg_tn == self.last_printed_tn:
            return
        with self.lock:
            # frequency
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0
            # std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) / n)
            # min and max
            max_delta = max(self.times)
            min_delta = min(self.times)
            self.last_printed_tn = self.msg_tn
        return rate, min_delta, max_delta, std_dev, n + 1


def get_parent_topics(topic, parent_topics=None, impl=False):
    global pubs, subs
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
    rospy.loginfo('subscribing {} topics'.format(len(topics)))
    hz_checkers = []
    for t in topics:
        rt = ROSTopicHz(window_size=-1)
        hz_checkers.append(rt)
        rospy.Subscriber(t, rospy.AnyMsg, rt.callback_hz)
    rospy.loginfo('subscribed {} topics'.format(len(topics)))
    # main loop
    while not rospy.is_shutdown():
        # collect topic hz stats
        show_topics, stats = [], []
        for i, rt in enumerate(hz_checkers):
            hz_stat = rt.get_hz()
            if hz_stat is None:
                continue
            rate, min_delta, max_delta, std_dev, _ = hz_stat
            show_topics.append(topics[i])
            stats.append([rate, min_delta, max_delta, std_dev])
        show_topics, stats = np.array(show_topics), np.array(stats)
        if stats.size == 0:
            rospy.logwarn('no messages')
        else:
            sort_indices = np.argsort(stats[:, 0])[::-1]
            show_topics, stats = show_topics[sort_indices], stats[sort_indices]
            stats = np.hstack((show_topics.reshape(-1, 1), stats)).tolist()
            # print stats result
            header = ['topic', 'rate', 'min_delta', 'max_delta', 'std_dev']
            table = Texttable(max_width=0)
            table.set_deco(Texttable.HEADER)
            table.add_rows([header] + stats)
            print(table.draw())
        # wait for next check
        rospy.rostime.wallsleep(1.0)


if __name__ == '__main__':
    NAME = 'find_topic_bottle_neck'
    ID = '/' + NAME
    rospy.init_node(NAME)
    master = rosgraph.Master(ID)
    state = master.getSystemState()
    publications, subscriptions, _ = state
    main()
