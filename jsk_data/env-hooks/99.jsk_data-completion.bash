#!/usr/bin/env bash


_jsk_data() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    opts=""

    if [ $COMP_CWORD -eq 1 ]; then
        opts="get ls put"
        COMPREPLY=( $(compgen -W "${opts}" ${cur}) )
        return 0
    fi

    case "$prev" in
      get|put)
        if [[ $cur =~ -+ ]]; then
          opts="--public"
        fi
        ;;
      ls)
        if [[ $cur =~ -+ ]]; then
          opts="--public --show-size --sort --reverse"
        fi
        ;;
      *) ;;
    esac
    COMPREPLY=( $(compgen -W "${opts}" ${cur} 2>/dev/null) ) && return 0
}
complete -o default -F _jsk_data jsk_data
