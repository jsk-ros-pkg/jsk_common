function _roscomplete_roschain {
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]]; then
        opts="-n -t"
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    else
        case ${COMP_WORDS[1]} in
            -n)
                opts=`rosnode list 2>/dev/null`
                COMPREPLY=($(compgen -W "$opts" -- ${arg}))
                ;;
            -t)
                opts=`rostopic list 2>/dev/null`
                COMPREPLY=($(compgen -W "$opts" -- ${arg}))
                ;;
        esac
    fi
}

complete -F "_roscomplete_roschain" "roschain"


