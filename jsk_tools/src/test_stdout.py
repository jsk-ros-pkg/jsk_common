#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import subprocess
import unittest

import rospy
import shlex


PKG = 'jsk_test_tools'
NAME = 'test_stdout'


class TestStdout(unittest.TestCase):
    def __init__(self, *args):
        super(TestStdout, self).__init__(*args)
        rospy.init_node(NAME)

    def test_stdout(self):
        command = rospy.get_param("~command")
        shell = rospy.get_param("~shell", False)
        if not shell:
            command = shlex.split(command)
        stdout = subprocess.check_output(command, shell=shell).decode()  # for python3 check_output returns b'spam'
        expected = rospy.get_param("~stdout", None)
        if expected:
            self.assertEqual(stdout.strip(), expected)
        for i, line in enumerate(stdout.splitlines()):
            expected = rospy.get_param("~stdout_line{}".format(i), None)
            if expected:
                self.assertEqual(line.strip(), expected)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestStdout, sys.argv)
