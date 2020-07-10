#!/usr/bin/env bash
filenames="";
for filename in ${@:1:$#}
do
    filenames=$filenames" "`pwd`/$filename;
done
echo $filenames

export ROS_MASTER_URI=http://localhost:11311
##for check args
##roslaunch --args rosbag_play pr2_play.launch bagfile_names:="$filenames";
roslaunch jsk_data baxter_play.launch bagfile_names:="$filenames";
