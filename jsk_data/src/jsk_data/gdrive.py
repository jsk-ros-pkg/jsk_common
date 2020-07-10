#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Wrapper module for gdrive command"""

import distutils.spawn
import os
import subprocess
import sys


# directory id in google drive of jsk
DIR_ID = '0B9P1L--7Wd2vUGplQkVLTFBWcFE'


def open_gdrive():
    url = 'https://drive.google.com/drive/u/1/folders/{}'.format(DIR_ID)
    if distutils.spawn.find_executable('xdg-open'):
        cmd = "xdg-open '{url}'".format(url=url)
    else:
        cmd = "gnome-open '{url}'".format(url=url)
    subprocess.call(cmd, shell=True)


def run_gdrive(args=None, stdout=True):
    if args is None:
        args = ''
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'jsk_data')
    config = os.path.join(pkg_ros_home, '.gdrive')
    cmd = 'rosrun jsk_data drive-linux-x64 --config {config} {args}'\
          .format(args=args, config=config)
    if stdout:
        return subprocess.check_output(cmd, shell=True)
    else:
        subprocess.call(cmd, shell=True)


def _init_gdrive():
    """This should be called before any commands with gdrive"""
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'jsk_data')
    config = os.path.join(pkg_ros_home, '.gdrive')
    if os.path.exists(os.path.join(config, 'token.json')):
        return
    if not os.path.exists(pkg_ros_home):
        os.makedirs(pkg_ros_home)
    run_gdrive(stdout=False)


def list_gdrive():
    _init_gdrive()
    args = '''list --query " '{id}' in parents" --noheader'''.format(id=DIR_ID)
    return run_gdrive(args=args)


def info_gdrive(id, only_filename=False):
    _init_gdrive()
    args = 'info --id {id}'.format(id=id)
    info = run_gdrive(args=args)
    if only_filename:
        return _info_gdrive_filename(stdout=info)
    return info


def _info_gdrive_filename(stdout):
    for line in stdout.splitlines():
        if line.startswith('Title: '):
            return line.split()[-1]


def upload_gdrive(filename):
    _init_gdrive()
    args = 'upload --file {file} --parent {id}'.format(file=filename,
                                                       id=DIR_ID)
    return run_gdrive(args=args)


def _find_id_by_filename(filename):
    if len(filename) > 40:
        filename = filename[:19] + '...' + filename[-18:]
    for line in list_gdrive().splitlines():
        file_id, title = line.split()[:2]
        if filename == title:
            return file_id
    else:
        sys.stderr.write('file not found: {0}\n'.format(filename))
        sys.stderr.write('Run `jsk_data ls --public` to find files.\n')
        return


def download_gdrive(filename):
    _init_gdrive()
    file_id = _find_id_by_filename(filename)
    args = 'download --id {}'.format(file_id)
    run_gdrive(args=args)


def delete_gdrive(filename):
    _init_gdrive()
    file_id = _find_id_by_filename(filename)
    args = 'delete --id {}'.format(file_id)
    run_gdrive(args=args)
