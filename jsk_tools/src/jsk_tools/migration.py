import re


class ROSMsgMigration(object):

    def __init__(self, from_pkg, to_pkg, msg_type, msg_files):
        self.from_pkg = from_pkg
        self.to_pkg = to_pkg
        self.msg_type = msg_type
        assert self.msg_type in ['msg', 'srv']
        self.msg_files = msg_files
        assert self.msg_files

    def migrate(self, fname):
        if [ext for ext in ['.c', '.cpp', '.h', '.hpp', '.hxx']
                if fname.endswith(ext)]:
            print(fname)
            self._migrate_cpp_file(fname)
        if fname.endswith('.py'):
            print(fname)
            self._migrate_py_file(fname)
        if [ext for ext in ['.l', '.lisp'] if fname.endswith(ext)]:
            print(fname)
            self._migrate_eus_file(fname)

    def _migrate_cpp_file(self, fname):
        replaced_lines = []
        f = open(fname, 'r')
        for line in f.readlines():
            for msg in self.msg_files:
                line = re.sub(r'#include <{}/{}.h>'.format(self.from_pkg, msg),
                              r'#include <{}/{}.h>'.format(self.to_pkg, msg),
                              line)
                line = re.sub(r'#include "{}/{}.h"'.format(self.from_pkg, msg),
                              r'#include "{}/{}.h"'.format(self.to_pkg, msg),
                              line)
                line = re.sub(r'{}::{}'.format(self.from_pkg, msg),
                              r'{}::{}'.format(self.to_pkg, msg),
                              line)
            replaced_lines.append(line)
        f.close()
        with open(fname, 'w') as f:
            f.writelines(replaced_lines)

    def _migrate_py_file(self, fname):
        replaced_lines = []
        f = open(fname, 'r')
        for line in f.readlines():
            for msg in self.msg_files:
                m = re.match(r'from {}.{} import(.*){}'
                             .format(self.from_pkg, self.msg_type, msg),
                             line)
                if m:
                    line = re.sub(
                        line[m.start():m.end()],
                        r'from {}.{} import{}{}'
                        .format(self.to_pkg, self.msg_type,
                                m.groups()[0], msg),
                        line)
                line = re.sub(
                    r'from {}.{} import \*'.format(self.from_pkg,
                                                   self.msg_type),
                    r'from {}.{} import *'.format(self.to_pkg, self.msg_type),
                    line)
                line = re.sub(
                    r'import {}.{}'.format(self.from_pkg, self.msg_type),
                    r'import {}.{}'.format(self.to_pkg, self.msg_type),
                    line)
                line = re.sub(
                    r'{}.{}.{}'.format(self.from_pkg, self.msg_type, msg),
                    r'{}.{}.{}'.format(self.to_pkg, self.msg_type, msg),
                    line)
            replaced_lines.append(line)
        f.close()
        with open(fname, 'w') as f:
            f.writelines(replaced_lines)

    def _migrate_eus_file(self, fname):
        replaced_lines = []
        f = open(fname, 'r')
        for line in f.readlines():
            for msg in self.msg_files:
                line = re.sub(
                    r'{}::{}'.format(self.from_pkg, msg),
                    r'{}::{}'.format(self.to_pkg, msg),
                    line)
            replaced_lines.append(line)
        f.close()
        with open(fname, 'w') as f:
            f.writelines(replaced_lines)
