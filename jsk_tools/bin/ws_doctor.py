#!/usr/bin/env python

import sys, os
from subprocess import check_output, CalledProcessError

# from termcolor import colored
def colored(string, color):
    colors = {
        'clear': '\033[0m',
        'black': '\033[30m',
        'red': '\033[31m',
        'green': '\033[32m',
        'yellow': '\033[33m',
        'blue': '\033[34m',
        'purple': '\033[35m',
        'cyan': '\033[36m',
        'white': '\033[37m'
        }
    if colors.has_key(color):
        return colors[color] + string + colors['clear']
    else:
        return string


from operator import add

def isROSWS():
    if os.environ.has_key("ROS_WORKSPACE"):
        ROS_WORKSPACE = os.environ["ROS_WORKSPACE"]
        return (ROS_WORKSPACE and 
                os.path.exists(os.path.join(ROS_WORKSPACE, ".rosinstall")))
    else:
        return False

def splitPathEnv(env_name):
    return env_name.split(":")

def estimateROSPackagePath(path):
    if path.endswith("devel"):
        return [path, os.path.join(path, "..", "src")]
    else:
        return [os.path.join(path, "share"), os.path.join(path, "stacks")]

def checkROSPackagePath():
    ROS_PACKAGE_PATH = splitPathEnv(os.environ["ROS_PACKAGE_PATH"])
    CMAKE_PREFIX_PATH = splitPathEnv(os.environ["CMAKE_PREFIX_PATH"])
    estimated_ros_package_path = [os.path.abspath(p) 
                                  for p in 
                                  reduce(add, [estimateROSPackagePath(c) 
                                               for c in CMAKE_PREFIX_PATH])]
    dubious_paths = []
    for p in ROS_PACKAGE_PATH:
        normalized_path = os.path.abspath(p)
        if not normalized_path in estimated_ros_package_path:
            if isROSWS():
                ROS_WORKSPACE = os.path.abspath(os.environ["ROS_WORKSPACE"])
                if not p.startswith(ROS_WORKSPACE):
                    dubious_paths.append(p)
    print colored("[ROS_PACKAGE_PATH]", "green")
    if dubious_paths:
        print "  ", colored("these path might be malformed: ", "red")
        for p in dubious_paths:
            print "    ", colored(p, "red")
    else:
        print "  ", colored("ROS_PACKAGE_PATH seems to be OK", "cyan")


def checkGitRepoDiff(git_path):
    os.chdir(git_path)
    output = check_output(["git", "status", "--short"])
    modified_files = []
    if output:
        for l in output.split("\n"):
            if l.startswith(" M"):
                modified_files.append(l)
    if modified_files:
        print colored("  %d files are modified" % (len(modified_files)), "red")
        for f in modified_files:
            print "    ", colored(f, "red")
            
def checkGitBranch(git_path):
    os.chdir(git_path)
    current_branch = check_output(["git", "rev-parse", "--abbrev-ref",
                                   "HEAD"]).strip()
    try:
        with open(os.devnull, "w") as devnull:
            current_tracking_branch = check_output(["git", "rev-parse",
                                                    "--abbrev-ref",
                                                    "--symbolic-full-name", "@{u}"],
                                                    stderr=devnull).strip()
    except CalledProcessError:
        current_tracking_branch = None
    remote_origin_head = check_output(["git", "rev-parse", "--abbrev-ref",
                                       "origin/HEAD"]).strip()
    if not current_tracking_branch:
        print "    ", colored("no tracking branch", "red")
    elif current_tracking_branch != remote_origin_head:
        print "    ", colored("the branch(%s) may not sync with %s" 
                              % (current_branch,
                              remote_origin_head), "red")
        
def checkGitRepo(git_path):
    print colored("[checking %s]" % git_path, "green")
    checkGitRepoDiff(git_path)
    checkGitBranch(git_path)
        
def checkWorkspace():
    workspace = None
    if isROSWS():
        workspace = os.environ["ROS_WORKSPACE"]
    else:
        workspace = os.path.abspath(os.path.join(splitPathEnv(os.environ["CMAKE_PREFIX_PATH"])[0], "..", "src"))
    git_repos = []
    for root, dirs, files in os.walk(workspace):
        if ".git" in dirs:                  
            if not [repo for repo in git_repos if root.startswith(repo)]: #ignore subdirs
                git_repos.append(root)
                checkGitRepo(root)
            

if __name__ == "__main__":
    checkROSPackagePath()
    checkWorkspace()
