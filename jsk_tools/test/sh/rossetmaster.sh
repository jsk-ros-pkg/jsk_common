#!/bin/bash

JSK_TOOLS_PATH=$(rospack find jsk_tools)
if [ "$JSK_TOOLS_PATH" = "/opt/ros/${ROS_DISTRO}/share/jsk_tools" ]; then
    . /opt/ros/${ROS_DISTRO}/setup.bash
else
    . `catkin locate --shell-verbs`
    if [ "$(rospack find jsk_tools)" = "$(catkin locate --install)/share/jsk_tools" ]; then
        # for install
        . "$(rospack find jsk_tools)"/catkin_env_hook/99.jsk_tools.sh
    else
        # for devel
        cd "$JSK_TOOLS_PATH" && . `catkin locate --devel`/etc/catkin/profile.d/99.jsk_tools.sh
    fi
fi


hostname=${1-"localhost"}
ros_port=${2-"11311"}
rossetmaster "$hostname" "$ros_port" > /dev/null 2>&1
echo "$ROS_MASTER_URI"
