#!/bin/sh

TOPICS="/joint_states /motor_states /lfsensor /rfsensor /lhsensor /rhsensor /imu /thermo_lleg /thermo_rleg /off_lfsensor /off_rfsensor /off_lhsensor /off_rhsensor /zmp /odom /urata_status"
# size unit is mega
# max is 100G
rosrun jsk_data rosbag_always.py --topics "$TOPICS" --size=100 --max-size=100000 --save-dir=$HOME/rosbags
