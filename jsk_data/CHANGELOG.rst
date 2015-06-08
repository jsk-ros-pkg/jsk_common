^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: mmurooka

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
