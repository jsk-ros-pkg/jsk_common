#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import re
import sys

import rosbag
import rosnode
import rospy
import rosgraph

ID = 'rosbag_for_rviz'


def is_node_connectable(master, node_name):
    node_api = rosnode.get_api_uri(master, node_name)
    if not node_api:
        return False
    try:
        rosnode.get_node_connection_info_description(node_api, master)
    except rosnode.ROSNodeIOException:
        return False
    return True


def get_rviz_nodes(master):
    rviz_nodes = []
    try:
        nodes = rosnode._sub_rosnode_listnodes().splitlines()
    except rosnode.ROSNodeIOException as e:
        sys.stderr.write('Error: {}\n'.format(e))
        sys.exit(1)
    for n in nodes:
        if re.search('/rviz', n) and is_node_connectable(master, n):
            rviz_nodes.append(n)
    return rviz_nodes


def main():
    master = rosgraph.Master(ID)

    for _ in xrange(3):
        rviz_nodes = get_rviz_nodes(master)
        if rviz_nodes:
            for n in rviz_nodes:
                print('Found rviz node: {}'.format(n))
            break
        sys.stderr.write('Error: No rviz nodes found! Waiting for 1 sec..\n')
        rospy.sleep(1)
    else:
        sys.stderr.write('Error: Timeout!\n')
        sys.exit(1)

    state = master.getSystemState()

    topics = []
    for n in rviz_nodes:
        subs = [t for t, l in state[1] if n in l]
        topics.extend(subs)
    if not topics:
        sys.stderr.write('Error: No topics found!\n')
        sys.exit(1)

    argv = sys.argv
    argv.insert(1, 'record')
    if len(argv) > 1:
        argv.extend(topics)
    rosbag.rosbagmain(argv)


if __name__ == '__main__':
    main()
