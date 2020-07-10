#!/usr/bin/env bash

OPTIONS="--clock"
FILENAMES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -*)
      OPTIONS="$OPTIONS $1"
      shift
      ;;
    *)
      FILENAMES="$FILENAMES $(readlink -f $1)"
      shift
      ;;
  esac
done

export ROS_MASTER_URI=http://localhost:11311
roslaunch jsk_data pr2_play.launch bagfile_names:="$FILENAMES" rosbag_option:="$OPTIONS"
