import fcntl
import os
import re
import subprocess

import rospy


device_re = re.compile(
    "Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$",  # NOQA
    re.I)


def reset_usb(device_path):
    if device_path is None:
        rospy.logwarn('device_path is not exists. '
                      'Please set device_path')
        return False
    fd = os.open(device_path, os.O_WRONLY)
    if fd < 0:
        rospy.logerr("Could not open {}".format(device_path))
        return False
    # Equivalent of the _IO('U', 20) constant in the linux kernel.
    USBDEVFS_RESET = ord('U') << (4 * 2) | 20
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    return True


def list_devices():
    df = subprocess.check_output("lsusb")
    devices = []
    if isinstance(df, bytes):
        df = df.decode('utf-8')
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'),
                                                          dinfo.pop('device'))
                devices.append(dinfo)
    return devices


def reset_usb_from_matched_pattern(pattern):
    """USB reset from name pattern.

    """
    devices = list_devices()
    filtered_devices = []
    for device in devices:
        if re.match(pattern, device['tag']):
            filtered_devices.append(device)
    for device in filtered_devices:
        rospy.loginfo(
            "Resetting USB device '{}' '{}'".format(device['tag'],
                                                    device['device']))
        reset_usb(device['device'])
