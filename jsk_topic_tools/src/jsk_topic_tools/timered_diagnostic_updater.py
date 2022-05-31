from distutils.version import LooseVersion

import diagnostic_updater
import pkg_resources
import rospy


class TimeredDiagnosticUpdater(object):

    def __init__(self, timer_duration):
        self.diagnostic_updater = diagnostic_updater.Updater()

        self.timer_kwargs = dict(
            period=timer_duration,
            callback=self.timer_callback,
            oneshot=False,
        )
        if (LooseVersion(pkg_resources.get_distribution('rospy').version) >=
                LooseVersion('1.12.0')):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            self.timer_kwargs['reset'] = True
        self.timer = None

    def start(self):
        if self.timer is None:
            self.timer = rospy.Timer(**self.timer_kwargs)

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None

    def set_hardware_id(self, name):
        self.diagnostic_updater.setHardwareID(name)

    def add(self, name, func):
        self.diagnostic_updater.add(name, func)

    def update(self):
        self.diagnostic_updater.update()

    def timer_callback(self, timer_event):
        self.update()
