#!/usr/bin/env zsh
# -*- mode: Shell-script; -*-

# source 99.jsk_data.sh from same directory as this file
_THIS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
if [ -f "$_THIS_DIR/99.jsk_data.sh" ]; then
    source "$_THIS_DIR/99.jsk_data.sh"
else
    # this is temporary code to avoid error caused by bug in ros/catkin
    # see https://github.com/jsk-ros-pkg/jsk_common/issues/885
    source "$_THIS_DIR/etc/catkin/profile.d/99.jsk_data.sh"
fi
unset _THIS_DIR
