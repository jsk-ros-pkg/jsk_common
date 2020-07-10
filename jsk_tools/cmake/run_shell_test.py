#!/usr/bin/env python

from __future__ import print_function

import argparse
from subprocess import Popen
from subprocess import PIPE
from subprocess import STDOUT
import sys

from rosunit import xml_results_file
from rosunit.junitxml import test_success_junit_xml
from rosunit.junitxml import test_failure_junit_xml


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('command', help='command to run')
    parser.add_argument('test_name', help='test name')
    parser.add_argument('result_file', help='test result file')
    parser.add_argument('--working-directory', help='working directory')
    args = parser.parse_args()

    cmd = args.command
    test_name = args.test_name
    result_file = args.result_file
    cwd = args.working_directory

    p = Popen(cmd, stdout=PIPE, stderr=STDOUT, shell=True, cwd=cwd)
    stdout, _ = p.communicate()
    ret_code = p.returncode

    passed = ret_code == 0

    print('...writing test results to', result_file)
    if passed:
        print('passed')
        with open(result_file, 'w') as f:
            f.write(test_success_junit_xml(test_name).decode())
    else:
        print('FAILURE:\n{0}'.format(stdout), file=sys.stderr)
        with open(result_file, 'w') as f:
            message = 'shell test with command [{0}] failed'.format(cmd)
            f.write(test_failure_junit_xml(test_name, message, stdout=stdout))
        print('wrote test file to [{0}]'.format(result_file))
        sys.exit(1)
