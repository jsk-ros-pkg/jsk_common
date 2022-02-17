from threading import Lock

import rospy


class VitalChecker(object):

    def __init__(self, dead_sec):
        self.dead_sec = dead_sec
        self.last_alive_time = rospy.Time.now()
        self.lock = Lock()

    def poke(self):
        with self.lock:
            self.last_alive_time = rospy.Time.now()

    def last_alive_time_relative(self):
        return (rospy.Time.now() - self.last_alive_time).to_sec()

    def is_alive(self):
        with self.lock:
            return self.last_alive_time_relative() < self.dead_sec

    def register_stat_info(self, stat, name):
        with self.lock:
            stat.add(name, '{} sec before'.format(
                self.last_alive_time_relative()))
