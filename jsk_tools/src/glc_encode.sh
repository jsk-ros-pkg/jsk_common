#!/bin/bash

usage_exit() {
    echo "Usage : $0 <glc_file> [--gtest file]" 1>&2
    exit 1
}
GLC_FILENAME=$1
MP4_FILENAME=${GLC_FILENAME%.glc}.mp4

GETOPT=`getopt -l text,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)       OUTFILE=$2      ; shift 2 ;;
  -h)   usage_exit ;;
  --)   shift ; break ;;
  *)   shift ; break ;;
  esac
done

OUTFILE=${OUTFILE/xml:/}
touch $OUTFILE
#
glc-play $GLC_FILENAME -o - -y 1 | ffmpeg -i - -sameq -y $MP4_FILENAME



