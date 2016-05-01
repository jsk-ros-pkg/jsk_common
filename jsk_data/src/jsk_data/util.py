#!/usr/bin/env python
# -*- coding: utf-8 -*-

import datetime
import os


def filename_with_timestamp(filename, sep=None):
    """Return filename with timestamp.

    @param filename: file path.
    @type filename: str
    @param spam: separator between timestamp and original filename.
    @type spam: str

        >>> filname_with_timestamp("./spam/ham.txt")
        ./spam/2015-11-14-17-42-00_ham.txt
    """
    head, tail = os.path.split(filename)
    now = datetime.datetime.now()
    if sep is None:
        sep = '_'
    format = '%Y-%m-%d-%H-%M-%S'  # length = 19
    try:
        datetime.datetime.strptime(tail[:19], format)
        return os.path.join(head, tail)
    except ValueError:
        return os.path.join(head, sep.join([now.strftime(format), tail]))


def google_drive_file_url(id, download=False):
    cmd = 'uc' if download else 'open'
    url = 'https://drive.google.com/{0}?id={1}'.format(cmd, id)
    return url
