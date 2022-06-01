#!/usr/bin/env python

import argparse
import os
import shutil

import termcolor

from jsk_rosbag_tools.compress import compress_bag_imgs


def main():
    parser = argparse.ArgumentParser(
        description='Convert Image messages '
        'to CompressedImage or CompressedDepthImage')
    parser.add_argument('input_bagfile', help='input bagfile path')
    parser.add_argument('--out', '-o', default='',
                        help='output bagfile path')
    parser.add_argument('--compressed-topics', nargs='*',
                        default=[],
                        help='this image topics are compressed')
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--no-progress-bar', action='store_true',
                        help="Don't show progress bar.")
    args = parser.parse_args()
    compressed_topics = [t.lstrip('/') for t in args.compressed_topics]

    input_bagfile = args.input_bagfile
    if len(args.out) == 0:
        basename = os.path.basename(input_bagfile)
        dirname = os.path.dirname(input_bagfile)
        filename, ext = os.path.splitext(basename)
        out = os.path.join(dirname, '{}-image-compressed{}'
                           .format(filename, ext))
    else:
        out = args.out
    compress_bag_imgs(input_bagfile, out,
                      compressed_topics=compressed_topics,
                      show_progress_bar=not args.no_progress_bar)
    if args.replace:
        termcolor.cprint('=> Replaced to {}'.format(out), 'green')
        shutil.move(out, input_bagfile)
    else:
        termcolor.cprint('=> Saved to {}'.format(out), 'green')


if __name__ == '__main__':
    main()
