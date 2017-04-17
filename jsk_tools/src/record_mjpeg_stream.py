#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import argparse
import getpass
import urlparse
import urllib2
import base64
import subprocess
import datetime

import rospy


def parse_args(args):
    parser = argparse.ArgumentParser(
                description='script to record web mjpeg stream')
    parser.add_argument('stream_url', help='ex. http://example.com/video.mjpeg')
    parser.add_argument('--ros', action='store_true',
                        help='to use as ROS node')
    parser.add_argument('--http-user', required=False,
                        help='username for basic auth')
    parser.add_argument('--http-password', required=False,
                        help='password for basic auth')
    timestamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    default_output_file = 'output_{0}.avi'.format(timestamp)
    parser.add_argument('-O', '--output-file', default=default_output_file,
                        help='default: output_XXX.avi')
    return parser.parse_args(args)


def add_auth_header(request, username, password):
    base64string = base64.encodestring(
        '{0}:{1}'.format(username, password))
    base64string.replace('\n', '')
    request.add_header('Authorization', 'Basic %s' % base64string)


def get_url_with_auth(url, username, password):
    o = urlparse.urlparse(url)
    url_with_auth = '{scheme}://{user}:{passwd}@{netloc}{path}'
    url_with_auth = url_with_auth.format(scheme=o.scheme, user=username,
                                        passwd=password, netloc=o.netloc,
                                        path=o.path)
    return url_with_auth


def main():
    args = parse_args(rospy.myargv(sys.argv[1:]))

    if args.ros:
        rospy.init_node('record_mjpeg_stream')

    req = urllib2.Request(args.stream_url)

    username = args.http_user
    password = args.http_password
    if (username is not None) and (password is not None):
        # add header for basic authorization
        add_auth_header(req, username, password)

    try:
        urllib2.urlopen(req)
    except urllib2.HTTPError, e:
        if e.code == 401:
            # ask for basic auth
            username = raw_input('username?: ')
            password = getpass.getpass('password?: ')
        else:
            raise urllib2.HTTPError, e

    stream_url = get_url_with_auth(args.stream_url, username, password)
    cmd = [
        'ffmpeg',
        '-f', 'mjpeg',
        '-i', stream_url,
        '-loglevel', '8',
        args.output_file,
        ]
    try:
        print('Please enter Ctr-C to stop recoding.')
        subprocess.call(cmd)
    except KeyboardInterrupt:
        print('Output file:', args.output_file)


if __name__ == '__main__':
    main()
