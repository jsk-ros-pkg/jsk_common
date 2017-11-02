#!/bin/sh
# -*- mode: Shell-script; -*-

rossetdefault() {
    local hostname=${1-"local"}
    local ros_port=${2-"11311"}
    if [ "$ROS_HOME" != "" ]; then
        local ros_home=$ROS_HOME
    else
        local ros_home="$HOME/.ros"
    fi
    mkdir -p $ros_home/jsk_tools
    echo -e "$hostname\n$ros_port" > $ros_home/jsk_tools/rosdefault
    rosdefault
}
rosdefault() {
    if [ "$ROS_HOME" != "" ]; then
        local ros_home=$ROS_HOME
    else
        local ros_home="$HOME/.ros"
    fi
    if [ -f $ros_home/jsk_tools/rosdefault ]; then
        local hostname="$(sed -n 1p $ros_home/jsk_tools/rosdefault)"
        local ros_port="$(sed -n 2p $ros_home/jsk_tools/rosdefault)"
    else
        local hostname="local"
        local ros_port="11311"
    fi
    if [ "$hostname" = "local" ]; then
        rossetlocal $ros_port
    else
        rossetmaster $hostname $ros_port
    fi
}

_update_prompt() {
    if [ "${WITHOUT_ROS_PROMPT}" = "" ]; then
        export WITHOUT_ROS_PROMPT="$PS1"
    fi
    local master_host=$(echo $ROS_MASTER_URI | cut -d\/ -f3 | cut -d\: -f1)
    if [ "$master_host" = "localhost" ]; then
        if echo $PS1 | grep "\[http://.*\]" > /dev/null
        then
            export PS1="${WITHOUT_ROS_PROMPT}"
        fi
    elif [ "$master_host" != "" ]; then
        local ros_prompt="[$ROS_MASTER_URI][$ROS_IP]"
        if [ "$CATKIN_SHELL" = "bash" ]; then
            export PS1="\[\033[00;31m\]$ros_prompt\[\033[00m\] ${WITHOUT_ROS_PROMPT}"
        elif [ "$CATKIN_SHELL" = "zsh" ]; then
            export PS1="%{$fg[red]%}$ros_prompt%{$reset_color%} ${WITHOUT_ROS_PROMPT}"
        else
            echo "unsupported shell"
        fi
    fi
}

rossetmaster() {
    local hostname=${1-"pr1040"}
    local ros_port=${2-"11311"}
    export ROS_MASTER_URI=http://$hostname:$ros_port
    if [ "$NO_ROS_PROMPT" = "" ]; then
        _update_prompt
    fi
    echo -e "\e[1;31mset ROS_MASTER_URI to $ROS_MASTER_URI\e[m"
}

rossetrobot() {
    echo -e "\e[1;31m *** rossetrobot is obsoleted, use rossetmaster ***\e[m"
    rossetmaster $@
}

rossetlocal() {
    local ros_port=${1-"11311"}
    rossetmaster localhost $ros_port
    if [ "$NO_ROS_PROMPT" = "" ]; then
        _update_prompt
    fi
}

rossetip_dev() {
    local device=${1-'(eth|wl|en|lo)[0-9]*'}
    export ROS_IP=`LANG=C ip -o -4 a | grep -E "^[0-9]+: $device" | tail -n1 | sed -e 's@^.*inet *\([0-9\.]*\).*$@\1@g'`
    export ROS_HOSTNAME=$ROS_IP
}

rossetip_addr() {
    local target_host=${1-"133.11.216.211"}
    # Check if target_host looks like ip address or not
    if [ "$(echo $target_host | sed -e 's/[0-9\.]//g')" != "" ]; then
        target_host_ip=$(timeout 0.01 getent hosts ${target_host} | cut -f 1 -d ' ')
        if [ "$target_host_ip" = "" ]; then
            echo -e "\e[1;31mCould not resolve ip from address. Subnet may be different\e[m"
            rossetip_dev
            return 0
        fi
        target_host=$target_host_ip
    fi
    export ROS_IP=$(ip -o -4 route get $target_host | sed -e 's/^.*src \([0-9.]\+\).*$/\1/g')
    export ROS_HOSTNAME=$ROS_IP
}

rossetip() {
    local device=${1-'(eth|wl|en|lo)[0-9]*'}
    if [[ $device =~ [0-9]+.[0-9]+.[0-9]+.[0-9]+ ]]; then
        export ROS_IP="$device"
    else
        export ROS_IP=""
        local master_host=$(echo $ROS_MASTER_URI | cut -d\/ -f3 | cut -d\: -f1)
        if [ "${master_host}" != "localhost" ]; then rossetip_addr ${master_host} ; fi
        if [ "${ROS_IP}" = "" ]; then rossetip_dev ${device}; fi
    fi
    export ROS_HOSTNAME=$ROS_IP
    if [ "${ROS_IP}" = "" ];
    then
        unset ROS_IP
        unset ROS_HOSTNAME
        echo -e "\e[1;31munable to set ROS_IP and ROS_HOSTNAME\e[m" >&2
        return 1
    else
        echo -e "\e[1;31mset ROS_IP and ROS_HOSTNAME to $ROS_IP\e[m"
    fi
    if [ "$NO_ROS_PROMPT" = "" ]; then
        _update_prompt
    fi
}

rosn() {
    if [ "$1" = "" ]; then
        select=$(rosnode list | percol | xargs -n 1 rosnode info | percol)
    else
        select=$(rosnode info $1 | percol)
    fi
    if [ "$select" != "" ]; then
        header=$(echo $select | awk '{print $1}')
        content=$(echo $select | awk '{print $2}')
        if [ "$header" = "*" ]; then
            rost $content
        else
            rosn
        fi
    fi
}

rost() {
    if [ "$1" = "" ]; then
        select=$(rostopic list | percol | xargs -n 1 rostopic info | percol)
    else
        select=$(rostopic info $1 | percol)
    fi
    if [ "$select" != "" ]; then
        header=$(echo $select | awk '{print $1}')
        content=$(echo $select | awk '{print $2}')
        if [ "$header" = "*" ]; then
            rosn $content
        elif [ "$header" = "Type:" ]; then
            rosmsg show $content | less
            rost
        else
            rost
        fi
    fi
}

restart_travis() {
  # Restart travis from command line
  if [ $# -lt 2 ]; then
    echo "usage: restart_travis <repo_slug> <job_id>"
    echo "example:"
    echo "  restart_travis jsk-ros-pkg/jsk_common 1258.2"
    return 1
  fi
  if [ -z $SLACK_TOKEN ]; then
    echo "Please set SLACK_TOKEN (see: https://api.slack.com/web)"
    return 1
  fi
  local slug job_id msg
  slug=$1
  job_id=$2
  msg="restart travis $slug $job_id"
  echo "sending... '$msg' -> #travis"
  echo $msg | slacker --channel travis --as-user
  if [ $? -eq 2 ]; then
    echo "Please upgrade slacker-cli" >&2
    return 1
  fi
}

rosbag_record_interactive() {
    rosbag record $(zenity --list --column topics $(rostopic list) --multiple --separator=' ')
}
