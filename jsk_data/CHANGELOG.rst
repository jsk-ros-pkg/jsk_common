^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.11 (2020-07-10)
-------------------
* [jsk_data] Add common rosbag_record and play file for fetch (`#1611 <https://github.com/jsk-ros-pkg/jsk_common/issues/1611>`_)

  * enable to give rosbag option to fetch_play.sh like pr2_play.sh
  * add option to launch rqt_bag and rviz in fetch_play.launch
  * [jsk_data/CMakeLists.txt] Add roslaunch_add_file_check for fetch's launch files
  * [jsk_data] Add fetch_play.sh
  * [jsk_data] Add fetch_play and fetch_record.launch

* use xdg-open for ubuntu16.04 and above (`#1638 <https://github.com/jsk-ros-pkg/jsk_common/issues/1638>`_)
* [jsk_data/common_record.launch] Add tf_static recordf( `#1641 <https://github.com/jsk-ros-pkg/jsk_common/issues/1641>`_)
* Fix for noetic build (`#1648 <https://github.com/jsk-ros-pkg/jsk_common/issues/1648>`_)

  * fix for python3, except, print ....
  * pytho3 dislike \d in regrex, src/test_topic_published.py:50:37: W605 invalid escape sequence '\d'
  * jsk_data/tests/test_data_collection_server.py use python3 as interpreter
     python code under scripts/ installed with catkin_python_install as http://wiki.ros.org/UsingPython3/SourceCodeChanges, to dynamically change shebangs on install time
    This installs tests/test_data_collection_server.py under CATKIN_PACKAGE_SHARE/tests, which outputs
    ```
    $ rosrun jsk_data test_data_collection_server.py
    [rosrun] You have chosen a non-unique executable, please pick one of the following:
    1) /home/user/devel/share/jsk_data/tests/test_data_collection_server.py
    2) /home/user/src/jsk_common/jsk_data/tests/test_data_collection_server.py
    ```
    we can ignore this message
  * fix jsk_data/src/jsk_data/cli.py:57:44: E741 ambiguous variable name 'l'
  * migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration
  * upgrade package.xml to format=3
  * use distutils.spawn

* Fix download_data.py for Python3f( `#1637 <https://github.com/jsk-ros-pkg/jsk_common/issues/1637>`_)

* [jsk_data] Skip extracting files which already existf( `#1626 <https://github.com/jsk-ros-pkg/jsk_common/issues/1626>`_)

  * Print message when skip extracting
  * Use isinstance() for checking file type
  * Skip extracting files which already exist

* pr2_play.launch: support indigo ( `#1620 <https://github.com/jsk-ros-pkg/jsk_common/issues/1620>`_)
* [download_data.py] generate error log when downloaded fils's md5 is incorrect (`#1610 <https://github.com/jsk-ros-pkg/jsk_common/issues/1610>`_)
* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Naoya Yamaguchi, Shingo Kitagawa, Yuto Uchimi, Iori Yanokura

2.2.10 (2018-11-03)
-------------------
* check if wget/gdown command exists, gdown is pip distributed so that we can not use this within de build farm (`#1609 <https://github.com/jsk-ros-pkg/jsk_common/issues/1609>`_)
* Contributors: Kei Okada

2.2.9 (2018-11-02)
------------------
* check if the pkg exists and path is writable (`#1608 <https://github.com/jsk-ros-pkg/jsk_common/issues/1608>`_)
  * fix http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__jsk_pcl_ros_utils__ubuntu_xenial_amd64__binary/132/console
* Contributors: Kei Okada

2.2.8 (2018-11-01)
------------------
* Fix installation destination (install node_scripts to CATKIN_PACKAGE_BIN_DESTINATION) (`#1604 <https://github.com/jsk-ros-pkg/jsk_common/issues/1604>`_)
* Enable method: all in test_data_collection_server.py (`#1600 <https://github.com/jsk-ros-pkg/jsk_common/issues/1600>`_)
  * Enable method: all in test_data_collection_server.py
    I missed this in https://github.com/jsk-ros-pkg/jsk_common/pull/1599#issuecomment-418374578.
* Add test for data_collection_server (`#1599 <https://github.com/jsk-ros-pkg/jsk_common/issues/1599>`_)
  * Re-enable test of data_collection_server with method=all
  * Add name to <test>
  * Disable test with static_image_publisher.py
  * Add test for data_collection_server
  * Rename to method: all from None since null is not supported in roslaunch
    ```
  ... logging to /home/wkentaro/.ros/log/rostest-hoop-18427.log
  [ROSUNIT] Outputting test results to /home/wkentaro/.ros/test_results/jsk_data/rostest-tests_data_collection_server.xml
  [Testcase: testtest_data_collection_server] ... ERROR!
  ERROR: cannot marshal None unless allow_none is enabled
  File "/usr/lib/python2.7/unittest/case.py", line 329, in run
  testMethod()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rostest/runner.py", line 120, in fn
  succeeded, failed = self.test_parent.launch()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rostest/rostest_parent.py", line 122, in launch
  return self.runner.launch()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 657, in launch
  self._setup()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 644, in _setup
  self._load_parameters()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/launch.py", line 338, in _load_parameters
  r  = param_server_multi()
  File "/usr/lib/python2.7/xmlrpclib.py", line 1006, in __call\_\_
  return MultiCallIterator(self.__server.system.multicall(marshalled_list))
  File "/usr/lib/python2.7/xmlrpclib.py", line 1243, in __call\_\_
  return self.__send(self.__name, args)
  File "/usr/lib/python2.7/xmlrpclib.py", line 1596, in __request
  allow_none=self.__allow_none)
  File "/usr/lib/python2.7/xmlrpclib.py", line 1094, in dumps
  data = m.dumps(params)
  File "/usr/lib/python2.7/xmlrpclib.py", line 638, in dumps
  dump(v, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
  f(self, value, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 719, in dump_array
  dump(v, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
  f(self, value, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 741, in dump_struct
  dump(v, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
  f(self, value, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 719, in dump_array
  dump(v, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 660, in __dump
  f(self, value, write)
  File "/usr/lib/python2.7/xmlrpclib.py", line 664, in dump_nil
  raise TypeError, "cannot marshal None unless allow_none is enabled"
    ```
  * add test for data_collection_server.test
* [jsk_data] Add re-download code. Fix `#1574 <https://github.com/jsk-ros-pkg/jsk_common/issues/1574>`_ (`#1589 <https://github.com/jsk-ros-pkg/jsk_common/issues/1589>`_)
  * [jsk_data] Add n_times option to try download
  * [jsk_data] Add download_data.py's test
  * [jsk_data] Add return value
  * [jsk_data] Add re-download code. Fix `#1574 <https://github.com/jsk-ros-pkg/jsk_common/issues/1574>`_

* update jsk_travis to 0.4.38 add lunar and melodic (`#1594 <https://github.com/jsk-ros-pkg/jsk_common/issues/1594>`_)
  * run pr2_play.launch test only when pr2_description_FOUND
* Fix mkdir in if isabs block in download_data (`#1593 <https://github.com/jsk-ros-pkg/jsk_common/issues/1593>`_)
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi, Yuto Uchimi, Iori Yanokura

2.2.7 (2018-06-27)
------------------
* jsk_data: chmod extraced files (`#1582 <https://github.com/jsk-ros-pkg/jsk_common/issues/1582>`_)
* [jsk_data] add option not to save in timestamp dir in data_collection_server.py (`#1578 <https://github.com/jsk-ros-pkg/jsk_common/issues/1578>`_)
* add timer save request in data_collection_server (`#1557 <https://github.com/jsk-ros-pkg/jsk_common/issues/1557>`_)
  * update sample data collection launch
  * add message_filers function in data_collection
  * add timer save request in data_collection_server
* Contributors: Shingo Kitagawa, Yuki Furuta

2.2.6 (2018-01-05)
------------------
* jsk_data: download_data.py: ensure chmod downloaded data if possible (`#1571 <https://github.com/jsk-ros-pkg/jsk_common/issues/1571>`_)
* jsk_data: download_data.py: Skip mkdir failures that can be caused by multiprocessing (`#1553 <https://github.com/jsk-ros-pkg/jsk_common/issues/1553>`_)
* Fix data_collection_server (`#1549 <https://github.com/jsk-ros-pkg/jsk_common/issues/1549>`_)
  * Sleep less time in data_collection_server.py
  * Return false response in data_collection_server
* Improve print information while download_data (`#1536 <https://github.com/jsk-ros-pkg/jsk_common/issues/1536>`_)
* [jsk_data][download_data.py] chmod decompressed data (`#1532 <https://github.com/jsk-ros-pkg/jsk_common/issues/1532>`_)
* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta

2.2.5 (2017-06-19)
------------------
* [jsk_data][download_data] support custom download dir / chmod  (`#1530 <https://github.com/jsk-ros-pkg/jsk_common/issues/1530>`_)
* Contributors: Yuki Furuta

2.2.4 (2017-06-14)
------------------
* [jsk_data][pr2_play.launch] replace doc to comment (`#1526 <https://github.com/jsk-ros-pkg/jsk_common/issues/1526>`_)
  * [jsk_data][pr2_play.launch] remove relay to c2 ns
  * [jsk_data][pr2_play.sh] support other rosbag arguments
* Fix bug for initialization of service server of data_collection_server (`#1525 <https://github.com/jsk-ros-pkg/jsk_common/issues/1525>`_)
  * Mode to save topics without request
    Modified:
    - jsk_data/node_scripts/data_collection_server.py
* Contributors: Kentaro Wada, Yuki Furuta

2.2.3 (2017-03-23)
------------------
* jsk_data/node_scripts/data_collection_server.py: Dump numpy.ndarray as npz file in data_collection_server.py (`#1508 <https://github.com/jsk-ros-pkg/jsk_common/issues/1508>`_)
  * Fix for flake8
  * Dump numpy.ndarray as npz file, For small size data using npz_compressed.
* Add my name to package.xml as a maintainer
* Contributors: Kentaro Wada

2.2.2 (2016-12-30)
------------------
* package.xml : Fix rosdep key: python-gdown -> python-gdown-pip
  According to https://github.com/ros/rosdistro/pull/13397
* jsk_data/download_data.py : Check if specified md5 has 32 charactors
* Contributors: Kentaro Wada

2.2.1 (2016-12-13)
------------------
* CMakeLists.txt : Strict rule of installing scripts
  Fix the part of `#1488 <https://github.com/jsk-ros-pkg/jsk_common/issues/1488>`_
* jsk_data/src/jsk_data/cli.py: Make stamping as optional in jsk_data (`#1486 <https://github.com/jsk-ros-pkg/jsk_common/issues/1486>`_)
  I found forcely chaning filename is a bit too strict.. ;)
* jsk_data/src/jsk_data/gdrive.py: Check if gdrive authorization has been successfully completed (`#1485 <https://github.com/jsk-ros-pkg/jsk_common/issues/1485>`_)
* jsk_data/data_collection_server.py:  set slop as rosparam and add warning in data_collection_server (`#1483 <https://github.com/jsk-ros-pkg/jsk_common/issues/1483>`_)
* jsk_data/data_collection_server.py:  Fix abs() for approx sync in data_collection_server.py (`#1477 <https://github.com/jsk-ros-pkg/jsk_common/issues/1477>`_)
* package.xml : Resolve dependency on python-gdown with rosdep (`#1481 <https://github.com/jsk-ros-pkg/jsk_common/issues/1481>`_)
* jsk_data/data_collection_server.py: fix typo in data_collection_server (`#1480 <https://github.com/jsk-ros-pkg/jsk_common/issues/1480>`_)
  * Fix visual indent and line length to follow pep8
  * fix indent in data_collection_server
* jsk_data/data_collection_server.py: add YAML topic savetype (`#1476 <https://github.com/jsk-ros-pkg/jsk_common/issues/1476>`_)
* jsk_data/data_collection_server.py: support non-header msg (`#1476 <https://github.com/jsk-ros-pkg/jsk_common/issues/1476>`_)
* Contributors: Kentaro Wada, Shingo Kitagawa

2.2.0 (2016-10-28)
------------------
* jsk_data/src/jsk_data/download_data.py: Create softlink for extracted files in download_data (`#1467 <https://github.com/jsk-ros-pkg/jsk_common/pull/1467>`_)
  - For multiple workspaces like in jenkins.
* Fix removing of symlink destination path (`#1469 <https://github.com/jsk-ros-pkg/jsk_common/pull/1469>`_)
* Contributors: Kentaro Wada

2.1.2 (2016-09-14)
------------------
* src/jsk_data/download_data.py : create path direcotory before download data and return if permission denied, catch resourceNotFound
* Contributors: Kei Okada

2.1.1 (2016-09-07)
------------------

2.1.0 (2016-09-06)
------------------

* record.launch : add bagfile_prefix arg, add machine argument (https://github.com/jsk-ros-pkg/jsk_common/pull/1437, https://github.com/jsk-ros-pkg/jsk_common/pull/1438)

  * jsk_data/CMakeLists.txt : pr2_record could not run on travis
  * [jsk_data] add machine argument for record.launch
  * [jsk_data] add bagfile_prefix arg for record.launch
  * jsk_data/CMakeLists.txt : check if baxter_description is installed
  * [jsk_data] add pr2_description to run_depend
  * [jsk_data] add xacro to run_depend for testing
  * [jsk_data] add baxter_description to run_depend for testing
  * [jsk_data] add bagfile_prefix arg for record.launch

* hrp2_play.launch use urdf model with hand for robot_description when  playing with hrp2. (`#1434 <https://github.com/jsk-ros-pkg/jsk_common/pull/1434>`_)
* pr2_play.launch: Remap /kinect_head topics to /kinect_head_c2 to play rosbag for pr2. (`#1431 <https://github.com/jsk-ros-pkg/jsk_common/pull/1431>`_)

* download_data.py: Add pkg_name for cache_dir to avoid data filename conflicts (`#1442 <https://github.com/jsk-ros-pkg/jsk_common/issues/1442>`_ )

  * Add pkg_name for cache_dir to avoid data filename conflicts
  * Support setting abspath for downloading data

* data_collection_server.py: Another saving type LabelImage of data_collection_server (`#1427 <https://github.com/jsk-ros-pkg/jsk_common/issues/1427>`_)

* camera_coords_change_trigger : Add trigger node for data collection by camera coords change  (`#1432 <https://github.com/jsk-ros-pkg/jsk_common/issues/1432>`_)
  Originally developped in
  https://github.com/furushchev/jsk_semantics_201607/blob/master/jsk_pr2_wandering/node_scripts/camera_coords_change_trigger.py.

* synchronize_republish.py : Synchronize properly with slop for slow topics  (`#1428 <https://github.com/jsk-ros-pkg/jsk_common/issues/1428>`_)

* Move README to sphinx docs for jsk_data package   (`#1433 <https://github.com/jsk-ros-pkg/jsk_common/issues/1433>`_)

* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Yuki Furuta

2.0.17 (2016-07-21)
-------------------
* Validate rosparams of data_collection_server.py
* Fix bug for new savetype YAML in data_collection_server.py
* Add YAML savetype to data_collection_server
* Add sample for data_collection_server in jsk_data
* Return saved message as TriggerResponse in data_collection_server
* Make params as optional for data_collection_server
* Change dynamically save_dir parameter in data_collection_server
* Contributors: Kentaro Wada

2.0.16 (2016-06-19)
-------------------

2.0.15 (2016-06-13)
-------------------
* Add data_collection_server.py
* Contributors: Kentaro Wada

2.0.14 (2016-05-14)
-------------------
* Add utility to download data (ex. test_data/trained_data)
* Fix url of google drive (view/download)
* Contributors: Kentaro Wada

2.0.13 (2016-04-29)
-------------------

2.0.12 (2016-04-18)
-------------------
* Omitted name of filename for gdrive go cli
* Contributors: Kentaro Wada

2.0.11 (2016-03-20)
-------------------

2.0.10 (2016-02-13)
-------------------
* [jsk_data] Fix deprecated arg in jsk_data command
* [jsk_data] exact_sync: true for publishing points
  Modified:
  - jsk_data/launch/kinect2_bridge_play.launch
* [jsk_data] Describe about pubopen and delete subcommands
  Modified:
  - jsk_data/README.md
* [jsk_data] Add pubopen subcommand to open GoogleDrive
  Modified:
  - jsk_data/src/jsk_data/cli.py
  - jsk_data/src/jsk_data/gdrive.py
* [jsk_data] Fix style and cleanup not used public_level
* [jsk_data] Support deleting file only public
* [jsk_data] Show fullname by pubinfo
* [jsk_data] Use --noheader option for listing
* [jsk_data] Download file from gdrive
* [jsk_data] Upload to gdrive with gdrive module
* [jsk_data] Use gdrive module for ls
* [jsk_data] Use gdrive wrapper for pubinfo
* [jsk_data] Add wrapper for drive command
* [jsk_data] Add drive binary for linux x64 v1.9.0 from prasmussen/gdrive
  see https://github.com/prasmussen/gdrive/releases/tag/1.9.0
  Added:
  - jsk_data/scripts/drive-linux-x64
* [jsk_data] Add playback launch for kinect2 using kinect2_bridge
  The reason I'd like to put this at this package is
  that installing kinect2_bridge package is not so easy.
* Contributors: Kentaro Wada

2.0.9 (2015-12-14)
------------------
* [jsk_data] Deepends on jsk_topic_tools
  Taking over https://github.com/jsk-ros-pkg/jsk_common/pull/1196
* Contributors: Ryohei Ueda

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
