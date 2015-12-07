^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.8 (2015-12-07)
------------------
* [jsk_data] Add roslint
* Contributors: Kentaro Wada

2.0.7 (2015-12-05)
------------------

2.0.6 (2015-12-02)
------------------

2.0.5 (2015-11-30)
------------------

2.0.4 (2015-11-25)
------------------
* [jsk_data/hrp2_rosbag_always.sh] Record capture points
* [jsk_data] Add stamp to file basename
* [jsk_data] Add flake8 code style check
* [jsk_data] Change path of tests for python package
* [jsk_data/launch] add urata_record.launch
* [jsk_data] Correctly gets selected file by percol
* [jsk_data] Describe about downloading large file from Google Drive
* [jsk_data] Add odom topics to be recorded by rosbag
* [jsk_data] Record PC voltage
* [jsk_data] Add shm_servo_state to rosbag always
* [jsk_data] Add rosbag_always.py document
* add new subscribe topic
* [jsk_data] Select filename at getting with jsk_data  Closes `#1141 <https://github.com/jsk-ros-pkg/jsk_common/issues/1141>`_
* [jsk_data] Documentation about `$ jsk_data` cli
* [jsk_data] Refactor: add cmd_pubinfo to __all\_\_
* [jsk_data] Select filename with percol in pubinfo
* [jsk_data] add camera parm to pr2_play.launch
* [jsk_data] Estimate filename if longer than 40
  Because gdrive does not return full title if it is longer than 40 Closes `#1155 <https://github.com/jsk-ros-pkg/jsk_common/issues/1155>`_
* [jsk_data] returning files does not work for zsh comp
* [jsk_data] Add file completion in bash
* [jsk_data] Refactor: indentation and comment
* [jsk_data] Display view url by pubinfo
* [jsk_data] Check existence of .ssh/config
* [jsk_data] Config key check when getting config from .ssh/config Closes `#1137 <https://github.com/jsk-ros-pkg/jsk_common/issues/1137>`_
* [jsk_data] Refactor cmd_put with google_drive_download_url
* [jsk_data] Add pubinfo subcommand
* [jsk_data] Remove old Makefile
* [jsk_data] Remove old jsk_data shell function
* [jsk_data] Add completion script for jsk_data
* [jsk_data] Add jsk_data command
* [jsk_data] Show size of files when listing remote bag files
* Add jsk_data function to handle data from anywhere
* [jsk_data] Record pgain and dgain in case something happens
* [jsk_tools] Use roslaunch internaly in rosbag_always.py in order to enable respawning
* [jsk_data/hrp2_rosbag_always.sh] Record more topics
* [jsk_tools] Record /urata_status topic in hrp2_rosbag_always.sh
* [jsk_data] Popup notification on desktop when removing a bag file
* [jsk_data] Handle bag files correctly with multiple ordered index
* [jsk_data/rosbag_always.py] Supress message about directory size and colorize message about removing bag files
* [jsk_data] Add more topics to record in hrp2_rosbag_always.sh
* Contributors: Kentaro Wada, Ryohei Ueda, Yusuke Oshiro, Yuto Inagaki, Eisoku Kuroiwa, Iori Yanokura

2.0.3 (2015-07-24)
------------------

2.0.2 (2015-07-07)
------------------

2.0.1 (2015-06-28)
------------------

2.0.0 (2015-06-19)
------------------
* Fix default ROBOT name
* Contributors: Kohei Kimura

1.0.72 (2015-06-07)
-------------------
* add  recording magnetometer
* Contributors: Ryo Terasawa

1.0.71 (2015-05-17)
-------------------
* [jsk_data] common_record.launch: Mkdir for saving rosbag file
* [jsk_data] Add image to all_image regex to common_record.launch
* Contributors: Kentaro Wada

1.0.70 (2015-05-08)
-------------------
* [jsk_data] add option in hrp2_play with multisense
* Contributors: Yu Ohara

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------
* [jsk_data] env value ARIES_USER will be default username to login aries
* [jsk_data] Add usage of KEYWORD for make large-list / small-list
* [jsk_data] Add KEYWORD to large-list/small-list target in Makefile
* Contributors: Kentaro Wada

1.0.67 (2015-05-03)
-------------------
* [jsk_data/rosbag_always.py] Remove old active file too
* [jsk_data] enable to select use_depth_image_proc or use_stereo_image_proc
* [jsk_data] add save_multisense parameter in hrp2_record.launch
* [jsk_data] add save_multisense parameter in common_record.launch
* [jsk_data] Save bags under ~/.ros directory
* Contributors: Kamada Hitoshi, Ryohei Ueda

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------
* [jsk_data] Utility script to save/load robot_description
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------
* [jsk_tilt_laser, jsk_data] Add multisense_play.launch to play multisene bag file
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------

1.0.61 (2015-02-11)
-------------------
* [jsk_data] catkinize
* Contributors: Ryohei Ueda

1.0.60 (2015-02-03 10:12)
-------------------------

1.0.59 (2015-02-03 04:05)
-------------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* Reuse isMasterAlive function across scripts which
  want to check master state
* modify output topic name again
* change output topic name into default
* add launch file for reconstruction of point cloud from multisense disparity image
* Contributors: Ryohei Ueda, Ryo Terasawa

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------
* Use ping with 10 seconds timeout to check master aliveness
* Contributors: Ryohei Ueda

1.0.55 (2014-12-09)
-------------------
* Check master is reachable before chcking master is alive
* Contributors: Ryohei Ueda

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------
* Fix rosbag to handle over 10 bags
* Contributors: Ryohei Ueda

1.0.51 (2014-10-20 16:01)
-------------------------

1.0.50 (2014-10-20 01:50)
-------------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* Add script to record rosbag always even if rosmaster is dead
* Contributors: Ryohei Ueda

1.0.47 (2014-10-08)
-------------------
* add pcds download option
* Contributors: Yuto Inagaki

1.0.46 (2014-10-03)
-------------------
* add baxter rosbag play
* Contributors: baxter

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26 09:17)
-------------------------

1.0.43 (2014-09-26 01:08)
-------------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------
* set save_all_image false in default
* add argument save_all_image to hrp2_record.launch. default is true.
* enable to set other_topic as argument
* Contributors: Masaki Murooka

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------
* add large-list and small-list to listup bag files in jsk_data server
* Contributors: Ryohei Ueda

1.0.38 (2014-09-13)
-------------------

1.0.37 (2014-09-08)
-------------------
* add use_xterm argument to pr2_play.launch
* add use_xterm argument to run rosbag with xterm
* Contributors: Ryohei Ueda

1.0.36 (2014-09-01)
-------------------
* Add a script to copy GOPRO movies to the server
* add common_record.launch and include it from hrp2_record.launch
  and pr2_record.launch
* add hrp2_record.launch hrp2_play.launch hrp2_play.sh
* Contributors: Ryohei Ueda, Satoshi Otsubo

1.0.35 (2014-08-16)
-------------------

1.0.34 (2014-08-14)
-------------------

1.0.33 (2014-07-28)
-------------------

1.0.32 (2014-07-26)
-------------------

1.0.31 (2014-07-23)
-------------------

1.0.30 (2014-07-15)
-------------------
* added codes to remove c2/c3 topics
* Contributors: Yu Ohara

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------
* add pkls Makefile option for random forest sklearn
* Contributors: Yuto Inagaki

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------
* I modified the program to use stream mode
* added programs for prosilica
* Contributors: Yu Ohara

1.0.22 (2014-05-22)
-------------------
* ignore large/ and small/ directories created by makefile
* Contributors: Ryohei Ueda

1.0.21 (2014-05-20)
-------------------
* update Makefile to decompress bag file when bag fiels is compressed
* more message on make large
* add rosbag option for set loop
* jsk_data: add KEYWORD features
* Contributors: Kei Okada, Yuto Inagaki

1.0.20 (2014-05-09)
-------------------

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19 23:29)
-------------------------

1.0.15 (2014-04-19 20:19)
-------------------------

1.0.14 (2014-04-19 12:52)
-------------------------

1.0.13 (2014-04-19 11:06)
-------------------------

1.0.12 (2014-04-18 16:58)
-------------------------

1.0.11 (2014-04-18 08:18)
-------------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------

1.0.6 (2014-04-07)
------------------

1.0.5 (2014-03-31)
------------------

1.0.4 (2014-03-29)
------------------
* jsk_data: add ssh -o StrictHostKeyChecking=no
* Contributors: Kei Okada

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* add "use_gui" argument
* enable to record gripper_command
* enable to record pressure-sensor
* add /tf when save_openni is true
* add jsk_data into jsk-ros-pkg for mainly rosbag
* Contributors: inagaki, iwaishi
