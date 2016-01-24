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


def main():
    master = rosgraph.Master(ID)
    rviz_nodes = []
    try:
        nodes = rosnode._sub_rosnode_listnodes().splitlines()
    except rosnode.ROSNodeIOException as e:
        sys.stderr.write('Error: {}\n'.format(e))
        sys.exit(1)
    for n in nodes:
        if re.search('/rviz', n) and is_node_connectable(master, n):
            rviz_nodes.append(n)
    if not rviz_nodes:
        sys.stderr.write('Error: rviz nodes not found!\n')
        sys.exit(1)

    # go through the master system state first
    try:
        state = master.getSystemState()
        pub_topics = master.getPublishedTopics('/')
    except socket.error:
        sys.stderr.write('Unable to communicate with master!')

    topics = []
    for n in rviz_nodes:
        subs = [t for t, l in state[1] if n in l]
        topics.extend(subs)

    argv = sys.argv
    argv.append('record')
    if len(argv) > 1:
        argv.extend(topics)
    rosbag.rosbagmain(argv)


if __name__ == '__main__':
    main()
