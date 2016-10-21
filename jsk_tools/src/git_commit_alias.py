#!/usr/bin/env python

"""Generate commit aliases for jsk-ros-pkg developers"""

import subprocess
from jsk_tools.github_lib import login_github



def main():
    gh = login_github()

    org = gh.get_organization('jsk-ros-pkg')
    for user in org.get_members():
        try:
            alias_name = name = user.name
            email = user.email
            if not email or email == "":
                raise Exception("No email specified")
            if len(name.split(" ")) > 0:
                alias_name = name.split(" ")[-1]
            alias_command = "commit-%s" % alias_name.lower()
            alias = "jsk-commit --author='%s <%s>'" % (name, email)
            subprocess.check_call(
                ["git", "config", "--global",
                "alias.%s" % alias_command, alias]
            )
            print "Added %s" % (alias_command)
        except Exception as e:
            print("Failed to generate alias for %s: %s" % (user.name, e))


if __name__ == '__main__':
    main()
