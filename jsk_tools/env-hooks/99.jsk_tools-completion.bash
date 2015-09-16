#!/usr/bin/env bash


# ZSH support
if [[ -n ${ZSH_VERSION-} ]]; then
    autoload -U +X bashcompinit && bashcompinit
fi

_restart_travis () {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    opts="euslisp/Euslisp
          euslisp/jskeus
          fkanehiro/hrpsys-base
          jsk-ros-pkg/geneus
          jsk-ros-pkg/euslisp-docs
          jsk-ros-pkg/jsk_3rdparty
          jsk-ros-pkg/jsk_common
          jsk-ros-pkg/jsk_control
          jsk-ros-pkg/jsk_demos
          jsk-ros-pkg/jsk_eus_pcl
          jsk-ros-pkg/jsk_model_tools
          jsk-ros-pkg/jsk_planning
          jsk-ros-pkg/jsk_pr2eus
          jsk-ros-pkg/jsk_recognition
          jsk-ros-pkg/jsk_robot
          jsk-ros-pkg/jsk_roseus
          jsk-ros-pkg/jsk_smart_apps
          jsk-ros-pkg/jsk_travis
          jsk-ros-pkg/jsk_visualization"

    COMPREPLY=( $(compgen -W "${opts}" ${cur}) )
    return 0
}
complete -F _restart_travis restart_travis