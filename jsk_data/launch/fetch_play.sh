#!/usr/bin/env bash

filenames="";
for filename in ${@:1:$#}
do
    filenames=$filenames" "$filename;
done
echo $filenames

export ROS_MASTER_URI=http://localhost:11311
roslaunch jsk_data fetch_play.launch bagfile_names:="$filenames";
