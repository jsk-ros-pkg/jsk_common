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


function rosview {
    if [[ $1 = "--help" ]]; then
        echo -e "usage: rosview [package] [file]\n\View a file within a package with pager."
        return 0
    fi
    if [[ -z $PAGER ]]; then
        roscat "$@" | less
    else
        roscat "$@" | $PAGER
    fi
}
compctl -f -x 'p[1]' -K "_roscomplete" - 'p[2]' -K _roscomplete_file -- "rosview"
