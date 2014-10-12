#!/bin/sh

TOPICS="/joint_states /motor_states /lfsensor /rfsensor /lhsensor /rhsensor /imu /thermo_lleg /thermo_rleg"
# size unit is mega
# max is 100G
rosrun jsk_data rosbag_always.py --topics "$TOPICS" --size=100 --max-size=100000 --save-dir=$HOME/rosbags
