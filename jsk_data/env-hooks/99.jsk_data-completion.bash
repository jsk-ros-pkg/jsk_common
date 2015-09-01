#!/usr/bin/env bash


_jsk_data()
{
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    opts=""

    case "$COMP_CWORD" in
      1)
        opts="get ls" ;;
      2)
        opts="large small" ;;
      *) ;;
    esac
    COMPREPLY=( $(compgen -W "${opts}" ${cur}) )
    return 0
}
complete -F _jsk_data jsk_data
