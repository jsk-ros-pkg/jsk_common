#!/usr/bin/env python

import diagnostic_msgs
import diagnostic_updater
from packaging import version
import pkg_resources
import rospy


def change_diagnostics(stat):
    current_state = True

    def _change_diagnostics(stat):
        if current_state:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         "Sensor is OK")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "Sensor is not OK.")
        current_state = not current_state
        stat.add("Current State", current_state)
        return stat
    return _change_diagnostics


class PseudoDiagnostics(object):

    def __init__(self, sensor_name, duration_time=5):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID(sensor_name)
        self.updater.add("pseudo_diagnostics", change_diagnostics)
        kwargs = dict(
            period=rospy.Duration(duration_time),
            callback=self.update_diagnostics,
            oneshot=False,
        )
        if version.parse(pkg_resources.get_distribution('rospy').version) \
                >= version.parse('1.12.0'):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            kwargs['reset'] = True
        self.timer = rospy.Timer(**kwargs)

    def update_diagnostics(self, event):
        self.updater.update()


if __name__ == '__main__':
    rospy.init_node('pseudo_diagnostics')
    pusedo_diagnotics = PseudoDiagnostics(
        sensor_name='sensor1',
        duration_time=5)
    pusedo_diagnotics = PseudoDiagnostics(
        sensor_name='sensor2',
        duration_time=10)
    rospy.spin()
