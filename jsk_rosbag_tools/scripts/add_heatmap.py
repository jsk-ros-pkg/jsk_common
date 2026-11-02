#!/usr/bin/env python
"""Add a numeric-array heatmap topic to a new ROS 1/ROS 2 bag offline."""

import argparse
import os

from jsk_ros1_ros2_compat.rosbag import ROS_VERSION
from jsk_rosbag_tools.add_heatmap import add_heatmap
from jsk_topic_tools.array_to_heatmap import HeatmapRenderer


def main():
    parser = argparse.ArgumentParser(
        description='Add a numeric MultiArray or single-channel Image topic as a '
                    'heatmap Image (bgr8) topic in a new ROS 1/ROS 2 bag.',
        epilog='Use the ROS environment matching the input bag format. '
               'The input bag is not modified. Set both color bounds to compare '
               'frames on a fixed scale; values are clipped for coloring only.')
    parser.add_argument(
        'input_bagfile',
        help='Input ROS 1 bag file or ROS 2 rosbag2 SQLite3 directory')
    parser.add_argument(
        '--out', '-o',
        help='Output path (must not exist). Default: <input-stem>--heatmap.bag '
             '(ROS 1) or <input-directory>--heatmap (ROS 2)')
    parser.add_argument(
        '--input-topic', required=True,
        help='Numeric MultiArray or single-channel Image topic to visualize')
    parser.add_argument(
        '--output-topic',
        help='New heatmap Image topic (must not exist). Default: INPUT_TOPIC/heatmap')
    parser.add_argument(
        '--rows', type=int, default=0,
        help='Number of rows; required with --cols for an empty MultiArray layout')
    parser.add_argument(
        '--cols', type=int, default=0,
        help='Number of columns; required with --rows for an empty MultiArray layout')
    parser.add_argument(
        '--frame-id', default='',
        help='Frame ID for MultiArray input (default: empty). Image headers '
             'are preserved')
    parser.add_argument(
        '--cell-size', type=int, default=64,
        help='Square cell size in pixels, at least 16 (default: 64)')
    parser.add_argument(
        '--hide-values', action='store_true',
        help='Hide cell values (shown by default)')
    parser.add_argument(
        '--precision', type=int, default=2,
        help='Decimal places from 0 to 10 (default: 2). Long values use '
             'scientific notation')
    parser.add_argument(
        '--colormap', default='jet',
        help='OpenCV colormap name, e.g. jet, hot, viridis (default: jet)')
    parser.add_argument(
        '--min-value', type=float,
        help="Lower color bound (default: each frame's finite minimum)")
    parser.add_argument(
        '--max-value', type=float,
        help="Upper color bound (default: each frame's finite maximum)")
    args = parser.parse_args()
    base = os.path.normpath(args.input_bagfile)
    default_out = (base + '--heatmap' if ROS_VERSION == 2 else
                   os.path.splitext(base)[0] + '--heatmap.bag')
    out = args.out or default_out
    output_topic = args.output_topic or args.input_topic.rstrip('/') + '/heatmap'
    try:
        renderer = HeatmapRenderer(
            cell_size=args.cell_size, show_values=not args.hide_values,
            precision=args.precision, colormap=args.colormap,
            min_value=args.min_value, max_value=args.max_value)
        count = add_heatmap(args.input_bagfile, out, args.input_topic,
                            output_topic, renderer, args.rows, args.cols,
                            args.frame_id)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print('Added {} images on {} to {}'.format(count, output_topic, out))


if __name__ == '__main__':
    main()
