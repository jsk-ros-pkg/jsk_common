#!/usr/bin/env python

import argparse
import os.path as osp

from jsk_rosbag_tools.bag_to_video import bag_to_video


def main():
    parser = argparse.ArgumentParser(description='rosbag to video')
    parser.add_argument('--out', '-o', default='',
                        help='output directory path or filename. '
                        'If more than one --image-topic are specified, '
                        'this will be interpreted as a directory name. '
                        'Otherwise this is the file name.')
    parser.add_argument('--fps', default=30,
                        type=int)
    parser.add_argument('--samplerate', '-r', type=int, help='sampling rate',
                        default='16000')
    parser.add_argument('--channels',
                        type=int, default=1, help='number of input channels')
    parser.add_argument('--audio-topic', type=str, default='/audio')
    parser.add_argument('--image-topic', type=str, default=[],
                        nargs='+', help='Topic name to extract.')
    parser.add_argument('--no-progress-bar', action='store_true',
                        help="Don't show progress bar.")
    parser.add_argument('input_bagfile')
    args = parser.parse_args()

    if len(args.out) == 0:
        args.out = osp.join(
            osp.dirname(args.input_bagfile),
            osp.splitext(osp.basename(args.input_bagfile))[0])
    input_bagfile = args.input_bagfile
    image_topic = None
    image_topics = None
    output_filepath = None
    output_dirpath = None
    if len(args.image_topic) == 1:
        image_topic = args.image_topic[0]
        output_filepath = args.out
    elif len(args.image_topic) > 1:
        image_topics = args.image_topic
        output_dirpath = args.out
    else:
        output_dirpath = args.out
    bag_to_video(input_bagfile,
                 output_filepath=output_filepath,
                 output_dirpath=output_dirpath,
                 image_topic=image_topic,
                 image_topics=image_topics,
                 fps=args.fps,
                 samplerate=args.samplerate,
                 channels=args.channels,
                 audio_topic=args.audio_topic,
                 show_progress_bar=not args.no_progress_bar)


if __name__ == '__main__':
    main()
