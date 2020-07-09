#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import optparse
import rospy

class TopicStatistics(object):
    def __init__(self, topic_name, duration, window_size=None):
        self.topic_name = topic_name
        self.duration = duration
        self.use_timer = True
        if window_size:
            self.use_timer = False
            self.window_size = window_size
        self.msgs = []
        self.start_time = None
        self.end_time = None

    def cb(self, msg):
        self.msgs.append(msg)
        if not self.use_timer and len(self.msgs) > self.window_size:
            self.stop()

    def start(self):
        print("start subscribing", self.topic_name)
        self.start_time = rospy.Time.now()
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.cb)
        if self.use_timer:
            print("waiting for", self.duration, "[sec]...")
            self.timer = rospy.Timer(rospy.Duration(self.duration), self.stop, True)
        else:
            print("waiting for", self.window_size, "msgs...")
        rospy.spin()

    def stop(self, _=None):
        self.end_time = rospy.Time.now()
        self.sub.unregister()
        self.show_result()
        rospy.signal_shutdown("finished measurement")

    def show_result(self):
        d = self.end_time - self.start_time
        msg_size = reduce(lambda a,b: a+b, map(lambda x: len(x._buff), self.msgs))
        print("received", len(self.msgs), "messages for", d.to_sec(), "[sec]")
        print("size amount:\t", msg_size / 1000.0, "[kB]")
        print("size avg:\t", msg_size / 1000.0 / len(self.msgs), "[kB/msg]")
        if self.use_timer:
            print("rate:\t\t", msg_size / 1000.0 / d.to_sec(), "[kB/sec]")

if __name__ == '__main__':
    p = optparse.OptionParser(usage="%prog [options] topic_name")
    p.add_option("-w", dest="window_size", type="int", default=None)
    p.add_option("-d", dest="duration", type="int", default=5)

    (opt, args) = p.parse_args(rospy.myargv())
    if len(args) != 2:
        p.print_help()
        exit(1)

    rospy.init_node("topic_statistics")
    t = TopicStatistics(topic_name=args[1],
                        window_size=opt.window_size,
                        duration=opt.duration)
    t.start()
