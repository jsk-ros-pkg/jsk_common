#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import shlex
import subprocess
import sys

import click
from jsk_tools.cltool import percol_select

from jsk_data.gdrive import delete_gdrive
from jsk_data.gdrive import download_gdrive
from jsk_data.gdrive import info_gdrive
from jsk_data.gdrive import list_gdrive
from jsk_data.gdrive import open_gdrive
from jsk_data.gdrive import upload_gdrive
from jsk_data.ssh import connect_ssh
from jsk_data.ssh import get_user_by_hostname
from jsk_data.util import filename_with_timestamp
from jsk_data.util import google_drive_file_url


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
        if public:
            lines = list_gdrive().splitlines()
            candidates = [line.split()[1] for line in lines]
        else:
            candidates = _list_aries_files()
        selected = percol_select(candidates)
        if len(selected) != 1:
            sys.stderr.write('Please select 1 filename.\n')
            sys.exit(1)
        query = selected[0]
        sys.stderr.write('Selected: {0}\n'.format(query))

    if public:
        download_gdrive(filename=query)
    else:
        cmd = 'rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no"\
            --bwlimit=100000 {usr}@{host}:{dir}/private/{q} .'
        cmd = cmd.format(usr=LOGIN_USER, host=HOST, dir=DATA_DIR, q=query)
        subprocess.call(shlex.split(cmd))


def _list_aries_files(query=None, ls_options=None):
    query = query or ''
    ls_options = ls_options or []
    with connect_ssh(HOST, LOGIN_USER) as ssh:
        cmd = 'ls {opt} {dir}/private/{q}'
        cmd = cmd.format(opt=' '.join(ls_options), dir=DATA_DIR, q=query)
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

    if public:
        if ls_options:
            sys.stderr.write(
                'WARNING: if public=True, ignores all ls options\n')
        sys.stdout.write(list_gdrive())
    else:
        print('\n'.join(_list_aries_files(query, ls_options)))


@cli.command(name='put', help='Upload file to aries.')
@click.argument('filename', required=True, type=click.Path(exists=True))
@click.option(
    '-p', '--public', is_flag=True,
    help=('Handle public files. It will be uploaded to Google Drive. '
          'Go https://drive.google.com/open?id=0B9P1L--7Wd2vUGplQkVLTFBWcFE'))
@click.option(
    '--stamped', is_flag=True,
    help='Rename file to with prefix of timestamp')
def cmd_put(filename, public, stamped):
    """Upload file to aries."""
    if stamped:
        filename_org = filename
        filename = filename_with_timestamp(filename)
        if filename_org != filename:
            print('Filename is being changed: {0} -> {1}'
                  .format(filename_org, filename))
            yn = raw_input('Are you sure?[Y/n]: ')
            if yn not in 'yY':
                sys.stderr.write('Aborted!')
                sys.exit(1)
            os.rename(filename_org, filename)

    if public:
        print('Uploading to Google Drive...')
        stdout = upload_gdrive(filename)
        for line in stdout.splitlines():
            if line.startswith('Title:'):
                filename = line.split(' ')[-1].strip()
            elif line.startswith('Id:'):
                file_id = line.split(' ')[-1].strip()
        print('Done.')
        print('You can download it by:')
        dl_url = google_drive_file_url(file_id, download=True)
        print('$ wget {url} -O {file}'.format(url=dl_url, file=filename))
    else:
        print('Uploading to aries...')
        cmd = 'rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no"\
            --bwlimit=100000 {file} {usr}@{host}:{dir}/private/'
        cmd = cmd.format(file=filename, usr=LOGIN_USER, host=HOST,
                         dir=DATA_DIR)
        subprocess.call(shlex.split(cmd))
        print('Done.')


@cli.command(name='pubinfo', help='Show public data info.')
@click.argument('filename', default='')
@click.option('-d', '--download-cmd', 'show_dl_cmd', is_flag=True,
              help='Print out download command')
def cmd_pubinfo(filename, show_dl_cmd):
    if not filename:
        candidates = list_gdrive().splitlines()
        selected = percol_select(candidates)
        if len(selected) != 1:
            sys.stderr.write('Please select 1 filename.\n')
            sys.exit(1)
        filename = selected[0].split()[1]

    # FIXME: gdrive does not return full title if it is longer than 40
    if len(filename) > 40:
        filename = filename[:19] + '...' + filename[-18:]

    stdout = list_gdrive()
    for line in stdout.splitlines():
        file_id, title = line.split()[:2]
        if filename == title:
            filename = info_gdrive(id=file_id, only_filename=True)
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


@cli.command(name='delete', help='Delete specified file.')
@click.option('-p', '--public', is_flag=True, help='Handle public files.')
@click.argument('filename', default='')
def cmd_delete(public, filename):
    """Delete specified file."""
    if not public:
        sys.stderr.write('ERROR: public=False is not supported\n')
        sys.exit(1)

    if not filename:
        # FIXME: gdrive does not return full title if it is longer than 40
        candidates = list_gdrive().splitlines()
        selected = percol_select(candidates)
        if len(selected) != 1:
            sys.stderr.write('Please select 1 filename.\n')
            sys.exit(1)
        filename = selected[0].split()[1]

    delete_gdrive(filename=filename)


@cli.command(name='pubopen', help='Go to see files on browser')
def cmd_pubopen():
    open_gdrive()
