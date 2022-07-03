#!/usr/bin/env python

import argparse
import os.path as osp

import termcolor

from jsk_rosbag_tools.bag_to_audio import bag_to_audio


def main():
    parser = argparse.ArgumentParser(description='rosbag to audio')
    parser.add_argument('--out', '-o', default='',
                        help='output filename. '
                        'If `--audio-topic`_info is exists, '
                        "you don't have to specify samplerate and channels.")
    parser.add_argument('--samplerate', '-r', type=int, help='sampling rate',
                        default='16000')
    parser.add_argument('--channels',
                        type=int, default=1, help='number of input channels')
    parser.add_argument('--audio-topic', type=str, default='/audio')
    parser.add_argument('input_bagfile')
    args = parser.parse_args()

    if len(args.out) == 0:
        args.out = osp.join(
            osp.dirname(args.input_bagfile),
            osp.splitext(osp.basename(args.input_bagfile))[0] + '.wav')

    audio_exists = bag_to_audio(
        args.input_bagfile, args.out,
        samplerate=args.samplerate,
        channels=args.channels,
        topic_name=args.audio_topic)
    if audio_exists is False:
        termcolor.cprint(
            '=> Audio topic is not exists', 'red')
    else:
        termcolor.cprint(
            '=> Audio saved to {}'.format(args.out), 'green')


if __name__ == '__main__':
    main()
