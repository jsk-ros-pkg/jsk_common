#!/usr/bin/env bash

filenames="";
for filename in ${@:1:$#}
do
    filenames=$filenames" "$filename;
done
echo $filenames

roslaunch jsk_data fetch_play.launch bagfile_names:="$filenames";
