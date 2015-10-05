#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import shlex
import subprocess
from contextlib import contextmanager

import click
import paramiko
from jsk_tools.cltool import percol_select

from .ssh import connect_ssh, get_user_by_hostname
from .util import filename_with_timestamp, google_drive_file_url


__all__ = ('cli', 'cmd_get', 'cmd_ls', 'cmd_put', 'cmd_pubinfo')


def _get_login_user(host):
    username = get_user_by_hostname(host)
    username = username or os.environ.get('SSH_USER')
    username = username or os.environ['USER']
    return username


HOST = 'aries.jsk.t.u-tokyo.ac.jp'
DATA_DIR = '/home/jsk/ros/data'
LOGIN_USER = _get_login_user(host=HOST)


CONTEXT_SETTINGS = dict(
    help_option_names=['-h', '--help'],
)
@click.group(context_settings=CONTEXT_SETTINGS)
def cli():
    pass


@cli.command(name='get', help='Download specified file.')
@click.option('-p', '--public', is_flag=True, help='Handle public files.')
@click.argument('query', default='')
def cmd_get(public, query):
    """Download specified file."""
    if not query:
        candidates = _list_aries_files(public=public)
        selected = percol_select(candidates)
        if len(selected) != 1:
            sys.stderr.write('Please select 1 filename.\n')
            sys.exit(1)
        filename = selected[0]

    public_level = 'public' if public else 'private'
    cmd = 'rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no"\
           --bwlimit=100000 {usr}@{host}:{dir}/{lv}/{q} .'
    cmd = cmd.format(usr=LOGIN_USER, host=HOST,
                     dir=DATA_DIR, lv=public_level, q=query)
    subprocess.call(shlex.split(cmd))


def _list_aries_files(public, query=None, ls_options=None):
    public_level = 'public' if public else 'private'
    query = query or ''
    ls_options = ls_options or []
    with connect_ssh(HOST, LOGIN_USER) as ssh:
        cmd = 'ls {opt} {dir}/{lv}/{q}'
        cmd = cmd.format(opt=' '.join(ls_options), dir=DATA_DIR,
                         lv=public_level, q=query)
        _, stdout, _ = ssh.exec_command(cmd)
        files = stdout.read().splitlines()
    return files


@cli.command(name='ls', help='Get list of files.')
@click.option('-p', '--public', is_flag=True, help='Handle public files.')
@click.argument('query', required=False)
@click.option('-s', '--show-size', is_flag=True, help='Display size.')
@click.option('--sort', help='Sort by ..',
              type=click.Choice(['time', 'size', 'extension', 'version']))
@click.option('-r', '--reverse', is_flag=True, help='Reverse the order.')
def cmd_ls(public, query, show_size, sort, reverse):
    """Get list of files."""
    public_level = 'public' if public else 'private'

    if query is None:
        query = ''

    ls_options = []
    if show_size:
        ls_options.append('--size')
        ls_options.append('--human-readable')
    if sort is not None:
        ls_options.append('--sort={0}'.format(sort))
    if reverse:
        ls_options.append('--reverse')

    print('\n'.join(_list_aries_files(public, query, ls_options)))


@cli.command(name='put', help='Upload file to aries.')
@click.option('-p', '--public', is_flag=True,
    help=('Handle public files. It will be uploaded to Google Drive. '
          'Go https://drive.google.com/open?id=0B9P1L--7Wd2vUGplQkVLTFBWcFE'))
@click.argument('filename', required=True, type=click.Path(exists=True))
def cmd_put(public, filename):
    """Upload file to aries."""
    public_level = 'public' if public else 'private'

    filename_org = filename
    filename = filename_with_timestamp(filename)
    if filename_org != filename:
        print('Filename is being changed: {0} -> {1}'\
                .format(filename_org, filename))
        yn = raw_input('Are you sure?[Y/n]: ')
        if yn not in 'yY':
            sys.stderr.write('Aborted!')
            sys.exit(1)
        os.rename(filename_org, filename)

    print('Uploading to aries...')
    cmd = 'rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no"\
           --bwlimit=100000 {file} {usr}@{host}:{dir}/{lv}/'
    cmd = cmd.format(file=filename, usr=LOGIN_USER, host=HOST,
                     dir=DATA_DIR, lv=public_level)
    subprocess.call(shlex.split(cmd))
    print('Done.')
    if public_level == 'private':
        sys.exit(0)

    print('Uploading to Google Drive...')
    with connect_ssh(HOST, LOGIN_USER) as ssh:
        cmd = '{dir}/scripts/upload-public-data.sh {dir}/public/{file}'
        cmd = cmd.format(dir=DATA_DIR, file=filename)
        _, stdout, stderr = ssh.exec_command(cmd)
        for line in stdout.readlines():
            if line.startswith('Title:'):
                filename = line.split(' ')[-1].strip()
            elif line.startswith('Id:'):
                file_id = line.split(' ')[-1].strip()
        sys.stderr.write(stderr.read())
    print('Done.')
    print('You can download it by:')
    dl_url = google_drive_file_url(file_id, download=True)
    print('$ wget {url} -O {file}'.format(url=dl_url, file=filename))


@cli.command(name='pubinfo', help='Show public data info.')
@click.argument('filename', default='')
@click.option('-d', '--download-cmd', 'show_dl_cmd', is_flag=True,
              help='Print out download command')
def cmd_pubinfo(filename, show_dl_cmd):
    if not filename:
        candidates = _list_aries_files(public=True)
        selected = percol_select(candidates)
        if len(selected) != 1:
            sys.stderr.write('Please select 1 filename.\n')
            sys.exit(1)
        filename = selected[0]

    with connect_ssh(HOST, LOGIN_USER) as ssh:
        cmd = '{dir}/scripts/list-public-data.sh'.format(dir=DATA_DIR)
        _, stdout, stderr = ssh.exec_command(cmd)
        stdout.next()  # drop header
        for line in stdout.readlines():
            file_id, title = line.split()[:2]
            # FIXME: gdrive does not return full title if it is longer than 40
            if len(filename) > 40:
                filename = filename[:19] + '...' + filename[-18:]
            if filename == title:
                break
        else:
            sys.stderr.write('file not found: {0}\n'.format(filename))
            sys.stderr.write('Run `jsk_data ls --public` to find files.\n')
            return

    dl_url = google_drive_file_url(file_id, download=True)
    if show_dl_cmd:
        info = 'wget {url} -O {file}'.format(url=dl_url, file=filename)
        sys.stdout.write(info)  # no new line for copy with pipe
    else:
        view_url = google_drive_file_url(file_id)
        info = '''\
Id: {id}
Filename: {file}
View URL: {view_url}
Download URL: {dl_url}'''.format(id=file_id, file=filename,
                                 view_url=view_url, dl_url=dl_url)
        print(info)
