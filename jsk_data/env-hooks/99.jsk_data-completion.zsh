#!/usr/bin/env zsh


# To use bash completion fiele for zsh
if [[ -n ${ZSH_VERSION-} ]]; then
    autoload -U +X bashcompinit && bashcompinit
fi


# source 99.jsk_data-completion.bash from same directory as this file
_THIS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
if [ -f "$_THIS_DIR/99.jsk_data-completion.bash" ]; then
    source "$_THIS_DIR/99.jsk_data-completion.bash"
else
    # this is temporary code to avoid error caused by bug in ros/catkin
    # see https://github.com/jsk-ros-pkg/jsk_common/issues/885
    source "$_THIS_DIR/etc/catkin/profile.d/99.jsk_data-completion.bash"
fi
unset _THIS_DIR
