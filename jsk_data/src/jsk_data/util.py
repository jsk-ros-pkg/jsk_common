#!/usr/bin/env python
# -*- coding: utf-8 -*-

import datetime


def filename_with_timestamp(filename, sep=None):
    now = datetime.datetime.now()
    if sep is None:
        sep = '_'
    format = '%Y-%m-%d-%H-%M-%S'  # length = 19
    try:
        datetime.datetime.strptime(filename[:19], format)
        return filename
    except ValueError:
        return sep.join([now.strftime(format), filename])


def google_drive_download_url(id):
    return 'https://drive.google.com/uc?id={0}&export=download'.format(id)
