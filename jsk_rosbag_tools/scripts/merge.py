#!/usr/bin/env python

import argparse

from jsk_rosbag_tools.merge import merge_bag


def main():
    parser = argparse.ArgumentParser(description='Merges two bagfiles.')
    parser.add_argument('--out', '-o',
                        type=str, help='name of the output file',
                        default=None, metavar="output_file")
    parser.add_argument(
        '--topics', '-t',
        type=str,
        help='topics which should be merged to the main bag',
        default=None)
    parser.add_argument('-i', help='reindex bagfile',
                        default=False, action="store_true")
    parser.add_argument(
        'main_bagfile',
        type=str,
        help='path to a bagfile, which will be the main bagfile')
    parser.add_argument(
        'bagfile',
        type=str,
        help='path to a bagfile which should be merged to the main bagfile')
    args = parser.parse_args()
    if args.topics is not None:
        args.topics = args.topics.split(',')
    merge_bag(args.main_bagfile,
              args.bagfile,
              outfile=args.out,
              topics=args.topics,
              reindex=args.i)


if __name__ == "__main__":
    main()
