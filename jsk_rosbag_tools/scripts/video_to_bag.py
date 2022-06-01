#!/usr/bin/env python

import argparse
from pathlib import Path

from jsk_rosbag_tools.video import video_to_bag


def main():
    parser = argparse.ArgumentParser(
        description='Convert video to bag.')
    parser.add_argument('inputvideo')
    parser.add_argument('--out', '-o', type=str,
                        help='name of the output bag file',
                        default=None, metavar="output_file")
    parser.add_argument('--topic-name', type=str,
                        default='/video/rgb/image_raw',
                        help='Converted topic name.')
    parser.add_argument('--compress', action='store_true',
                        help='Compress Image flag.')
    parser.add_argument('--no-progress-bar', action='store_true',
                        help="Don't show progress bar.")
    args = parser.parse_args()

    video_path = Path(args.inputvideo)
    if args.out is None:
        args.out = video_path.with_suffix('.bag')

    outfile = Path(args.out)
    pattern = str(video_path.parent / (video_path.stem + "_%i.bag"))
    index = 0
    while outfile.exists():
        outfile = Path(pattern % index)
        index += 1
    video_to_bag(
        video_path, outfile,
        args.topic_name,
        compress=args.compress,
        show_progress_bar=not args.no_progress_bar)


if __name__ == '__main__':
    main()
