#!/usr/bin/env bash
# -*- mode: Shell-script; -*-

# source 99.jsk_tools.sh from same directory as this file
_THIS_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_THIS_DIR/99.jsk_tools.sh"


function rosview {
    if [[ $1 = "--help" ]]; then
       echo -e "usage: rosview [package] [file]\n\nView a file within a package with pager."
       return 0
    fi
    if [[ -z $PAGER ]]; then
        roscat "$@" | less
    else
        roscat "$@" | $PAGER
    fi
}
complete -F "_roscomplete_file" "rosview"
