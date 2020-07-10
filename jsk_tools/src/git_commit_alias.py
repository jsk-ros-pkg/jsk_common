#!/usr/bin/env python

# Generate commit aliases for jsk-ros-pkg developers
import subprocess
from pygithub3 import Github

from getpass import getpass
user = raw_input('GitHub User name: ')
pw = getpass('Password: ')

gh = Github(login=user, password=pw)
result = gh.orgs.members.list('jsk-ros-pkg')
for page in result:
    for member in page:
        user = gh.users.get(member.login)
        try:
            name = user.name
            alias_name = name
            email = user.email
            if not email or email == "":
                raise Exception("No email specified")
            if len(alias_name.split(" ")) > 0:
                alias_name = name.split(" ")[-1]
            alias_command = "commit-%s" % alias_name.lower()
            alias = "jsk-commit --author='%s <%s>'" % (name, email)
            subprocess.check_call(["git", "config", "--global", 
                                   "alias.%s" % alias_command,
                                   alias])
            print("Added %s" % (alias_command))
        except:
            print("Failed to generate alias for %s" % (member.login))

