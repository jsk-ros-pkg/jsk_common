#!/bin/sh

TOPICS="/joint_states /joint_states_appended /motor_states /motor_states/abs_angle /motor_states/abs_cur_angle_diff \
/motor_states/board_vdd /motor_states/board_vin /motor_states/cur_angle \
/motor_states/motor_current /motor_states/motor_outer_temp /motor_states/motor_temp \
/motor_states/ref_angle /motor_states/servo_alarm \
/motor_states/pgain /motor_states/dgain \
/urata_status /shm_servo_state \
/off_lfsensor /off_rfsensor /off_lhsensor /off_rhsensor \
/lfsensor /rfsensor /lhsensor /rhsensor \
/act_capture_point /ref_capture_point \
/zmp /odom /imu \
/viso_odom /viso_odom_integrated /biped_odom_integrated /integrated_odom \
/thermo_lleg /thermo_rleg \
/inside_pc_voltage_publisher \
/multisense_local/resize_1_8/left/image_rect_color/throttle

"

# size unit is mega
# max is 100G
rosrun jsk_data rosbag_always.py --topics "$TOPICS" --size=100 --max-size=100000 --save-dir=$HOME/rosbags
