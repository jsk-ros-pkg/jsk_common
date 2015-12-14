#!/bin/bash

PROGNAME=$(basename $0)
VERSION=`rosversion jsk_tools`

usage() {
    echo "Usage: $PROGNAME FILE [OPTIONS]"
    echo "  This script is wrapper for roslaunch supporting rostest."
    echo
    echo "Options:"
    echo "  -h, --help"
    echo "  -v, --version"
    echo " --gtest_output=xml:[PATH] output path"
    echo
    exit 1
}

for OPT in "$@"; do
    case "$OPT" in
        '-h'|'--help' )
            usage
            ;;
        '-v'|'--version' )
            echo $VERSION
            exit 0
            ;;
        --gtest_output=xml:* )
            OUTPUT=`echo $1 | sed 's/--gtest_output=xml://g'`
            shift 1
            ;;
        '--'|'-' )
            shift 1
            PARAM+=( "$@" )
            break
            ;;
        -*)
            echo "$PROGNAME: illegal option -- '$(echo $1 | sed 's/^-*//')'" 1>&2
            exit 1
            ;;
        *)
            if [[ ! -z "$1" ]] && [[ ! "$1" =~ ^-+ ]]; then
                PARAM+=( "$1" )
                shift 1
            fi
            ;;
    esac
done

if [ "$PARAM" = "" ]; then
    echo "You must specify FILE."
    usage
fi

exec rosrun roslaunch roslaunch-check $PARAM -o $OUTPUT
