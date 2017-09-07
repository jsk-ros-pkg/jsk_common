import __builtin__

import getpass
import sys

import github


def raw_input(prompt=None):
    if prompt:
        sys.stderr.write(str(prompt))
    return __builtin__.raw_input()


gh = None


def login_github():
    global gh
    if gh is None:
        username = raw_input('GitHub username: ')
        password = getpass.getpass('Password: ', stream=sys.stderr)
        gh = github.Github(username, password)
    return gh
