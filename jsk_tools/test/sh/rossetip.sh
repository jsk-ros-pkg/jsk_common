#!/bin/bash

JSK_TOOLS_PATH=$(rospack find jsk_tools)
if [ "$JSK_TOOLS_PATH" = "/opt/ros/${ROS_DISTRO}/share/jsk_tools" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    source `catkin locate --shell-verbs`
    if [ "$(rospack find jsk_tools)" = "$(catkin locate --install)/share/jsk_tools" ]; then
        # for install
        source "$(rospack find jsk_tools)"/catkin_env_hook/99.jsk_tools.sh
    else
        # for devel
        cd "$JSK_TOOLS_PATH" && source `catkin locate --devel`/etc/catkin/profile.d/99.jsk_tools.sh
    fi
fi


ECHO_OUTPUT="$1"

function getent () {
    echo "127.0.0.1       localhost"
}

function ip () {
    # overwrite ip command.
    echo "$ECHO_OUTPUT"
}

rossetip_addr > /dev/null 2>&1
echo $ROS_IP
