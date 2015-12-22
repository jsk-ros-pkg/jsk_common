#!/usr/bin/env zsh
# -*- mode: Shell-script; -*-

# source 99.jsk_tools.sh from same directory as this file
_THIS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
if [ -f "$_THIS_DIR/99.jsk_tools.sh" ]; then
    source "$_THIS_DIR/99.jsk_tools.sh"
else
    # this is temporary code to avoid error caused by bug in ros/catkin
    # see https://github.com/jsk-ros-pkg/jsk_common/issues/885
    source "$_THIS_DIR/etc/catkin/profile.d/99.jsk_tools.sh"
fi
unset _THIS_DIR


# PR at upstream: https://github.com/ros/ros/pull/99
function rosview {
    local arg
    if [[ $1 = "--help" ]]; then
        echo -e "usage: rosview [package] [file]\n\View a file within a package with pager."
        return 0
    fi
    _roscmd ${1} ${2}
    if [[ -z $PAGER ]]; then
        less ${arg}
    else
        $PAGER ${arg}
    fi
}
compctl -f -x 'p[1]' -K "_roscomplete" - 'p[2]' -K _roscomplete_file -- "rosview"
