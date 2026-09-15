#!/usr/bin/env python

import argparse
import os

import termcolor
from tqdm import tqdm

from jsk_ros1_ros2_compat.rosbag import nanoseconds_from_sec
from jsk_ros1_ros2_compat.rosbag import open_bag
from jsk_ros1_ros2_compat.rosbag import stamp_from_nanoseconds


def main():
    parser = argparse.ArgumentParser(
        description='Convert tf_static to tf and save it as a rosbag')
    parser.add_argument('input_bagfile', help='input bagfile path')
    parser.add_argument('--out', '-o', default='',
                        help='output bagfile path')
    parser.add_argument('--no-progress-bar', action='store_true',
                        help="Don't show progress bar.")
    args = parser.parse_args()

    input_bagfile = args.input_bagfile
    if len(args.out) == 0:
        basename = os.path.basename(input_bagfile)
        dirname = os.path.dirname(input_bagfile)
        filename, ext = os.path.splitext(basename)
        out = os.path.join(dirname, '{}--tf_static--converted{}'
                           .format(filename, ext))
    else:
        out = args.out

    input_bag = open_bag(input_bagfile)
    tf_static_messages = list(input_bag.read_messages(('/tf_static')))

    with open_bag(out, 'w') as outbag:
        if args.no_progress_bar is False:
            progress = tqdm(total=input_bag.get_message_count())
        for topic, msg, t in input_bag:
            # update the progress with a post fix
            if args.no_progress_bar is False:
                progress.update(1)
                progress.set_postfix(time=t)
            if topic == '/tf':
                for tsm in tf_static_messages:
                    for tsmtr in tsm.message.transforms:
                        tsmtr.header.stamp = stamp_from_nanoseconds(t)
                    outbag.write('/tf', tsm.message,
                                 t - nanoseconds_from_sec(0.1))
            outbag.write(topic, msg, t)
    termcolor.cprint('=> Saved to {}'.format(out), 'green')


if __name__ == '__main__':
    main()
