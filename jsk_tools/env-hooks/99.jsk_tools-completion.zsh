#!/usr/bin/env zsh
# -*- mode: Shell-script; -*-

_THIS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
if [ -f "$_THIS_DIR/99.jsk_tools-completion.bash" ]; then
    source "$_THIS_DIR/99.jsk_tools-completion.bash"
else
    # this is temporary code to avoid error caused by bug in ros/catkin
    # see https://github.com/jsk-ros-pkg/jsk_common/issues/885
    source "$_THIS_DIR/etc/catkin/profile.d/99.jsk_tools-completion.bash"
fi
unset _THIS_DIR
