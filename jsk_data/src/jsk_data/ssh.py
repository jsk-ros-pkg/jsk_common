#!/usr/bin/env python
# -*- coding: utf-8 -*-

from contextlib import contextmanager
import os

import paramiko


def connect_ssh(host, username=None, password=None):
    return _connect_ssh_context(host, username, password)


@contextmanager
def _connect_ssh_context(host, username, password):
    try:
        ssh = paramiko.SSHClient()
        ssh.load_host_keys(os.path.expanduser('~/.ssh/known_hosts'))
        ssh.connect(host, username=username, password=password)
        yield ssh
    finally:
        ssh.close()


def get_user_by_hostname(hostname):
    ssh_config_file = os.path.expanduser('~/.ssh/config')
    if not os.path.exists(ssh_config_file):
        return
    with open(ssh_config_file) as f:
        ssh_config = paramiko.util.parse_ssh_config(f)
    for entry in ssh_config._config:
        if 'config' not in entry:
            continue
        config = entry['config']
        if config.get('hostname') == hostname:
            return config.get('user')
