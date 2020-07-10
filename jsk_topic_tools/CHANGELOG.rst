^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.11 (2020-07-10)
-------------------
* [jsk_topic_tools] check nodelet version>=1.9.10 (`#1647 <https://github.com/jsk-ros-pkg/jsk_common/issues/1647>`_)
* [jsk_topic_tools/scripts/pose_stamped_publisher.py] fix orientation bug (`#1649 <https://github.com/jsk-ros-pkg/jsk_common/issues/1649>`_)
* Fix for noetic build (`#1648 <https://github.com/jsk-ros-pkg/jsk_common/issues/1648>`_)

  * fix for python3, except, print ....
  * fix print(), Exception as e for python3
  * fox for boost 1.67 (20.04)
  * migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration
  * upgrade package.xml to format=3

* call ros::param::get before set not to overwrite (`#1643 <https://github.com/jsk-ros-pkg/jsk_common/issues/1643>`_)

  * run test_standalone_complexed_nodelet.test
  * add test code for standalone_complexed_nodelet
  * call ros::param::get before set not to overwrite

* [jsk_topic_tools/SynchronizedThrottle] Reset sync policy in destructor (`#1640 <https://github.com/jsk-ros-pkg/jsk_common/issues/1640>`_)

* [jsk_topic_tools] import _pickle as pickle for python3 (`#1636 <https://github.com/jsk-ros-pkg/jsk_common/issues/1636>`_)

  * add comment in log_utils
  * import _pickle as pickle for python3, cpickle is no more used in python3

* add SoundRequest.volume for kinetic (`#1635 <https://github.com/jsk-ros-pkg/jsk_common/issues/1635>`_)
* Create tf.TransformListener before run timerf( `#1634 <https://github.com/jsk-ros-pkg/jsk_common/issues/1634>`_)

  * Assign listener varaible before run timer and the callback in order, not to lookup listener variable before it is assigned.

* [jsk_tools] Add --ping-trials option to roscore_regardless.pyf( `#1632 <https://github.com/jsk-ros-pkg/jsk_common/issues/1632>`_)

  * Sometimes ping is not stable. `--ping-trials` option enables roscore_regardless.py to verify host computer of rosmaster is alive by multi-times ping commands.

* [deprecated_relay] print warning message only when relayed topic is subscribed (`#1624 <https://github.com/jsk-ros-pkg/jsk_common/issues/1624>`_)

  * print warn only when the msg is subscribed
  * print warn only once in starting

* [jsk_tools] Add --timeout option to roscore_regardless.py (`#1622 <https://github.com/jsk-ros-pkg/jsk_common/issues/1622>`_)
* standalone_complexed_nodelet: add `params` key for each nodelet (`#1614 <https://github.com/jsk-ros-pkg/jsk_common/issues/1614>`_)

  * Add --timeout option to change timeout duration of ping command towards rosmaster computer.
  * --timeout option defaults to 10 seconds.

* jsk_nodelet: fix overwritting find_package(boost) (`#1618 <https://github.com/jsk-ros-pkg/jsk_common/issues/1618>`_)
* synchronized_throttle: add some more infos (`#1615 <https://github.com/jsk-ros-pkg/jsk_common/issues/1615>`_)
* stealth_relay_nodelet: fix error double free or corruption (fasttop) (`#1613 <https://github.com/jsk-ros-pkg/jsk_common/issues/1613>`_)

  * update standalone_complexed_nodelet sample launch
  * standalone_complexed_ndoelet: support params tag

* Contributors: Furushchev, Kei Okada, Ryo Koyama, Ryohei Ueda, Shingo Kitagawa, Yuki Furuta, Iory Yanokura

2.2.10 (2018-11-03)
-------------------

2.2.9 (2018-11-02)
------------------

2.2.8 (2018-11-01)
------------------
* Fix to install 'scripts' directory (`#1604 <https://github.com/jsk-ros-pkg/jsk_common/issues/1604>`_)
* Add reset to Timer in ConnectionBasedTransport (`#1597 <https://github.com/jsk-ros-pkg/jsk_common/issues/1597>`_)
  * Check if >=kinetic to pass reset arg to Timer

* Add test for data_collection_server (`#1599 <https://github.com/jsk-ros-pkg/jsk_common/issues/1599>`_)
  * Stop using cv2 in static_image_publisher.py
    To fix
    https://github.com/jsk-ros-pkg/jsk_common/pull/1599#issuecomment-417908500
  * Add reset to Timer in ConnectionBasedTransport
    To fix below:
    ```
  [ERROR] [1535796247.786932, 1535792085.063646]: [/get_heightmap] [sleep] ROS time moved backwards: 1.407559397s
  Exception in thread Thread-4:
  Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
  self.run()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 226, in run
  r.sleep()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 103, in sleep
  sleep(self._remaining(curr_time))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 164, in sleep
  raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
  ROSTimeMovedBackwardsException: ROS time moved backwards
  Exception in thread Thread-4:
  Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
  self.run()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 226, in run
  r.sleep()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 103, in sleep
  sleep(self._remaining(curr_time))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 164, in sleep
  raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
  ROSTimeMovedBackwardsException: ROS time moved backwards
  ^C[image_view-9] killing on exit
  [tile_image-8] killing on exit
  [get_heightmap/output/depth_view-7] killing on exit
  [get_heightmap-6] killing on exit
  [heightmap_frame_publisher-5] killing on exit
  [bbox_to_tf-4] killing on exit
  [bbox_array_to_bbox-3] killing on exit
  [rosbag_play-2] killing on exit
  [rosout-1] killing on exit
  [master] killing on exit
  shutting down processing monitor...
  ... shutting down processing monitor complete
  done
    ```
* [jsk_topic_tools] Fixed use_warn option (`#1592 <https://github.com/jsk-ros-pkg/jsk_common/issues/1592>`_)
* use PROJECT_NAME instad of __NODENAME_PREFIX (RANDOM) (`#1591 <https://github.com/jsk-ros-pkg/jsk_common/issues/1591>`_)
  * https://github.com/jsk-ros-pkg/jsk_common/pull/1586/files#r207146300
* jsk_topic_tools/cmake/nodelet.cmake: add random prefix before _single  (``#1586 <https://github.com/jsk-ros-pkg/jsk_common/issues/1586>`_)
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi, Yuto Uchimi, Iori Yanokura

2.2.7 (2018-06-27)
------------------
* Add warnNoRemap to ConnectionBasedNodelet (`#1538 <https://github.com/jsk-ros-pkg/jsk_common/issues/1538>`_)
  * add version_gte 1.9.11 for nodelet
* jsk_topic_tools: add option to display diagnostic messages on warning level (`#1585 <https://github.com/jsk-ros-pkg/jsk_common/issues/1585>`_)
  * jsk_topic_tools: add option to set diangostic level
    jsk_topic_tools: update doc for jsk_topic_tools nodelet classes
* Add #include <boost/format.hpp> (`#1584 <https://github.com/jsk-ros-pkg/jsk_common/issues/1584>`_)
* jsk_topic_tools: add synchronized_throttle (`#1579 <https://github.com/jsk-ros-pkg/jsk_common/issues/1579>`_)
  * jsk_topic_tools: add synchronized_throttle
  * Add warnNoRemap to ConnectionBasedNodelet
* Fix roscore regardless (`#1576 <https://github.com/jsk-ros-pkg/jsk_common/issues/1576>`_)
  * jsk_topic_tools: fix isMasterAlive to work
* Contributors: Yuki Furuta, Kentaro Wada, Laurenz

2.2.6 (2018-01-05)
------------------
* jsk_topic_tools: stealth_relay_nodelet: support MessageEvent (`#1572 <https://github.com/jsk-ros-pkg/jsk_common/issues/1572>`_)
* jsk_topic_tools: stealth_relay add options as dynamic_reconfigure (`#1568 <https://github.com/jsk-ros-pkg/jsk_common/issues/1568>`_)
  * jsk_topic_tools: test_stealth_relay: disable updating dynamic reconfigure
  * jsk_topic_tools: test_stealth_relay: update timeout
  * jsk_topic_tools: stealth_relay: add deprecation warning
  * jsk_topic_tools: add options as dynamic_reconfigure

* jsk_topic_tools: connection_based_nodelet: fix typo in advertiseCamera (`#1558 <https://github.com/jsk-ros-pkg/jsk_common/issues/1558>`_)
* jsk_topic_tools: add stealth_relay for silently subscribing topic (`#1544 <https://github.com/jsk-ros-pkg/jsk_common/issues/1544>`_)
* Validate implementation of child class of ConnectionBasedTransport (`#1556 <https://github.com/jsk-ros-pkg/jsk_common/issues/1556>`_)
  * Check if publishers exist to avoid implementation failures
  * Use ABCMeta to avoid unexpected usage of ConnectionBasedTransport
    Someone use this class without any subscriptions,
    and in that case this class should not be used in general.
* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta

2.2.5 (2017-06-19)
------------------

2.2.4 (2017-06-14)
------------------
* [jsk_topic_tools][LightweightThrottle] dynamic change update_rate (`#1514 <https://github.com/jsk-ros-pkg/jsk_common/pull/1514>`_)
  *  [jsk_topic_tools][lightweight_throttle] support jump back in time

* [jsk_topic_tools][connection_based_nodelet] add isSubscribed method (`#1523 <https://github.com/jsk-ros-pkg/jsk_common/pull/1523>`_)
* Test disconnection in test_connection.py (`#1520 <https://github.com/jsk-ros-pkg/jsk_common/pull/1520>`_)
  - modified:   test/test_connection.py
  - https://github.com/jsk-ros-pkg/jsk_common/pull/1520#issuecomment-298151270
* [jsk_topic_tools][connection_based_nodelet] warn if onInitPostProcess is not called (`#1513 <https://github.com/jsk-ros-pkg/jsk_common/pull/1513>`_)
* Contributors: Kentaro Wada, Yuki Furuta

2.2.3 (2017-03-23)
------------------
* jsk_topic_tools/scripts/tf_to_transform.py: Use different value for duration and rate in tf_to_transform.py (`#1509 <https://github.com/jsk-ros-pkg/jsk_common/issues/1509>`_)
  * Rate can be 50 - 100 for example, but duration should be ~1 [s] even
    so. In previous implementation, the duration will be 1/100 - 1/50 [s]
    and it is too small to resolve tf.
  * Fix for flake8
* Contributors: Kentaro Wada

2.2.2 (2016-12-30)
------------------

2.2.1 (2016-12-13)
------------------
* add tf_to_transform node (`#1482 <https://github.com/jsk-ros-pkg/jsk_common/issues/1482>`_)
* Contributors: Shingo Kitagawa

2.2.0 (2016-10-28)
------------------
* include/jsk_topic_tools/log_utils.h : JSK_ROS_XXX logging macros are not necessary just recently. Its feature is already covered by ROSCONSOLE_FORMAT environmental variable. http://wiki.ros.org/rosconsole#Console_Output_Formatting  (`#1461 <https://github.com/jsk-ros-pkg/jsk_common/issues/1461>`_)

  * Stop using deprecated jsk_topic_tools/log_utils.h (`#1470 <https://github.com/jsk-ros-pkg/jsk_common/issues/1470>`_)
    see
    - https://github.com/jsk-ros-pkg/jsk_common/pull/1462
    - https://github.com/jsk-ros-pkg/jsk_common/issues/1461
  * Fix too many warnings about JSK_ROS_XXX (`#1468 <https://github.com/jsk-ros-pkg/jsk_common/issues/1468>`_)
  * [jsk_topic_tools] Deprecate JSK log macros and show warning (`#1462 <https://github.com/jsk-ros-pkg/jsk_common/issues/1462>`_)
    * feedback: Use ROS_WARN
    * Deprecate JSK log macros and show warning
     See https://github.com/jsk-ros-pkg/jsk_common/issues/1461

* [jsk_topic_tools/scripts/tf_to_pose.py] add rate param. (`#1457 <https://github.com/jsk-ros-pkg/jsk_common/issues/1457>`_)

* Contributors: Kentaro Wada, Masaki Murooka

2.1.2 (2016-09-14)
------------------

2.1.1 (2016-09-07)
------------------
* Fix missing installation of jsk_topic_tools_test_nodelet.xml
* Contributors: Kentaro Wada

2.1.0 (2016-09-06)
------------------
* [synchronize_republish.py] Republish after approximate synchronization (`#1443 <https://github.com/jsk-ros-pkg/jsk_common/issues/1443>`_)

  * Add sample for synchronize_republish.py
  * Add script to publish statid image for sample/testing
  * Republish after approxiamte synchronization
  * Refactor synchrnoze_republish.py (making it pythonic)

* Refactor CMake files (`#1447 <https://github.com/jsk-ros-pkg/jsk_common/issues/1447>`_)

  * Use project exported library for linking target library
  * Add ::test namespace to avoid conflicts of nodelet class name
  * Rename to have log_utils in the filename
  * Nodelet should be have suffix of _nodelet

* add JSK_NODELET_LOG_THROTTLE (`#1446 <https://github.com/jsk-ros-pkg/jsk_common/issues/1446>`_)

  * [jsk_topic_tools] add test for JSK_NODELET_LOG
  * [jsk_topic_tools/src/log_utils.h] add THROTTLE to JSK_NODELET_LOG

* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta

2.0.17 (2016-07-21)
-------------------
* Add JSK_ROS_XXX_THROTTLE, JSK_ROS_XXX_STREAM_THROTTLE
* Contributors: Kentaro Wada

2.0.16 (2016-06-19)
-------------------
* Fix unreasonable test name of test_log_utils.cpp
* Add test for getFunctionName
* Use JSK_NODELET_WARN in connection_based_nodelet
* Show only func name in JSK_XXX log utils
* Contributors: Kentaro Wada

2.0.15 (2016-06-13)
-------------------
* add parameter for selecting MultiThread callback or SingleThread callback
* Test LoggingThrottle
* Implement logXXX_throttle
* Support async in is_synchronized
* Install only usable *.test files
* Test and documentize tf_to_pose.py
* Transform tf to pose and publish it
* Contributors: Kentaro Wada, Yohei Kakiuchi

2.0.14 (2016-05-14)
-------------------
* Show node name and func name by log_utils
* Contributors: Kentaro Wada

2.0.13 (2016-04-29)
-------------------

2.0.12 (2016-04-18)
-------------------
* Set flag of subscribed even when always_subscribe
  Modified:
  - jsk_topic_tools/src/connection_based_nodelet.cpp
* Show test condition for 'scripts/is_synchronized'
* Support timeout and exit fastly
* Add method of wait_for_sync in 'scripts/is_synchronized'
* Set queue_size as 100
* Fix unregistering of the subscribers
* Exit with exit code to represent the synchronization
* Use rostime to check synchronization
* Contributors: Kentaro Wada

2.0.11 (2016-03-20)
-------------------
* [jsk_topic_tools] Set property is_initialized
  Modified:
  - jsk_topic_tools/src/jsk_topic_tools/transport.py
* Contributors: Kentaro Wada

2.0.10 (2016-02-13)
-------------------
* [jsk_topic_tools] Fix topic to advertise by advertiseImage and advertiseCamera
* Support jsk_tilt_laser and jsk_topic_tools on OS X
* [jsk_topic_tools] Find Boost with quiet option always in order
  to avoid link error with boost programs options.
* [jsk_topic_tools] Add script to synchronize timestamp of topics
  and republish them for visualization
* [jsk_topic_tools/scripts/bag2csv.py] support flatten option in bag2csv.
* [jsk_topic_tools] Return to avoid segfault when --inout opt
  Modified:
  - jsk_topic_tools/cmake/single_nodelet_exec.cpp.in
* [jsk_topic_tools] Add symlink to doc
* [jsk_topic_tools] Add tool to check synchronized topics
  Added:
  - jsk_topic_tools/scripts/is_synchronized.py
* [jsk_topic_tools] Stop using roslint as test
  Need https://github.com/jsk-ros-pkg/jsk_travis/pull/219
  Modified:
  jsk_topic_tools/CMakeLists.txt
* Update maintainer of jsk_network_tools and jsk_topic_tools
* Contributors: Kentaro Wada, Masaki Murooka, Ryohei Ueda

2.0.9 (2015-12-14)
------------------
* [jsk_topic_tools] Fix typo: test -> text in rosping_existence.py
* [jsk_topic_tools] Stop using enum34 and use just int
* [jsk_topic_tools/rosping_existence] Add ~speak_text parameter to customization
* [jsk_topic_tools/log_utils] Fix include guard.
  Define warnNoRemap in include guard section.
* [jsk_topic_tools] Find Boost_LIBRARIES once nodelet.cmake
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.8 (2015-12-07)
------------------
* [jsk_topic_tools] Add roslint_cpp for src/log_utils.cpp
* [jsk_topic_tools] Add roslint_python
* [jsk_topic_tools] Refactor CMakeLists.txt by moving rostest find_package
* [jsk_topic_tools] Fix for pep8
* [jsk_topic_tools/ConnectionBasedNodelet] Support image_transport.
  Add advertiseImage and advertiseCamera.
  closes `#1198 <https://github.com/jsk-ros-pkg/jsk_common/issues/1198>`_
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.7 (2015-12-05)
------------------
* Use ccache if installed to make it fast to generate obj file
* [jsk_topic_tools] Fix linking for boost_program_options
* [jsk_topic_tools] Add sample launch file for standalone_complexed_nodelet
* [jsk_topic_tools] Show input/output topics with --inout opt
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.6 (2015-12-02)
------------------

2.0.5 (2015-11-30)
------------------
* [jsk_topic_tools] Rename _util.py -> _utils.py
* [jsk_topic_tools] Also fix import in test_name_util.py: name_util -> name_utils
* [jsk_topic_tools] Fix renamed module import in log_utils: name_util -> name_utils
* Contributors: Iori Kumagai, Kentaro Wada, Yuto Inagaki

2.0.4 (2015-11-25)
------------------
* [jsk_topic_tools] test related things in CATKIN_ENABLE_TESTING block
* [jsk_topic_tools] Test warnNoRemap  Closes `jsk-ros-pkg/jsk_recognition#1322 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1322>`_
* [jsk_topic_tools/rosping_existence] Speak dead nodes
* Use gcc -z defs to check undefined symbols in shared objects  Related to https://github.com/jsk-ros-pkg/jsk_recognition/pull/1330
* [jsk_topic_tools] Retry to 3 times
* [jsk_topic_tools] Test rosparam_utils.cpp with gtest
* [jsk_topic_tools] Test warn_no_remap
* [jsk_topic_tools] Test jsk_topic_tools.log_util
* [jsk_topic_tools] Test jsk_topic_tools.name_util
* [jsk_topic_tools] add_library src/log_utils.cpp
* build_depend -> test_depend roscpp_tutorials
* Reasonable connection num for connection_based_nodelet
* [jsk_topic_tools] Use retry for <test> tag
* Refactor test_hz_measure.py as good example
* Refactor test_connection.py as good example
* Refactor: test_block.py as good example
* [jsk_topic_tools] display input/output by --inout
* [jsk_topic_tools] Fix style (indent)
* [jsk_topic_tools] Follow name rule *_utils.py
* [jsk_topic_tools] warnNoRemap for cpp nodes
* Generate Documentation for jsk_topic_tools
* [jsk_topic_tools] Function to warn with no remappings
* [jsk_topic_tools] Correctly return instance
* [jsk_topic_tools] Retry test max to 3 times
* [jsk_topic_tools] add topic_statistics.py
* [jsk_topic_tools] Correctly unsubscribe with multiple publishers
* [jsk_topic_tools] ``add_rostest`` problem should be fixed in latest catkin For https://github.com/jsk-ros-pkg/jsk_common/pull/1178#issuecomment-147396447
* [jsk_topic_tools] Describe about ~always_subscribe in warning
* [jsk_topic_tools] Add ~always_subscribe param for ConnectionBasedTransport
* [jsk_topic_tools] Correctly set connection status
* [jsk_topic_tools] Add log_utils.py
* [jsk_topic_tools] Add python-enum34 as run_depend
* [jsk_topic_tools] List depends in alphabetical order
* [jsk_topic_tools] Test ConnectionBasedTransport
* [jsk_topic_tools] Test ConnectionBasedNodelet with rostest
* [jsk_topic_tools] Rename to test_connection_based_nodelet.test
* [jsk_topic_tools] Python ConnectionBasedTransport
* [jsk_topic_tools] Utility to publish PoseStamped with given static transformation
* [jsk_topic_tools/ConnectionBasedNodelet] Read `verbose_connection` as well as `~verbose_connection`
* [jsk_topic_tools/ConnectionBasedNodelet] `~verbose_connection` parameter to print verbose messages about connection
* [jsk_topic_tools] Ros error for rosparam type conversion
* [jsk_topic_tools] Warn when no connection in a few sec Closes `#1132 <https://github.com/jsk-ros-pkg/jsk_common/issues/1132>`_  The warning message should be write with ROS_INFO,  for no many warning when running with roslaunch.
* [jsk_topic_tools] Supress output messages from testing
* [jsk_topic_tools] Depends on roscpp and rostime explicitly
* [jsk_topic_tools] Faster implementation of test_topic_compare.py by removing magic sleep
* [jsk_topic_tools/ConnectionBasedNodelet] Add latch option to advertise template method
* [jsk_topic_tools/LightweightThrottle] Clean-up codes and added some comments
* [jsk_topic_tools] Add readme about standalone_complexed_nodelet
* [jsk_topic_tools] check /run_id param to know roscore is restarted or not
* [jsk_topic_tools/standalone_complexed_nodelet] Fix handling of reampping name resolvance
* [jsk_topic_tools] Add space after [functionname]
* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Ryohei Ueda

2.0.3 (2015-07-24)
------------------
* [jsk_topic_tools] Install missing executables
* [jsk_topic_tools/standalone_complexed_nodelet] Support if and unless
  fields and read parameter from ~nodelet_%lu as well as ~nodelet
* [jsk_topic_tools] Introduce new nodelet manager called
  standalone_complexed_nodelet.
  It reads nodelet clients from rosparam and launch them. It is a general
  model for nodelet like stereo_image_proc. It does not need different
  processes for manager/clients
* [jsk_topic_tools] Make advertise template method critical section in
  order to avoid race condition between advertise and connectionCallback
* [jsk_topic_tools] Add StringRelay nodelet to test DiagnosticNodelet class
* Contributors: Ryohei Ueda

2.0.2 (2015-07-07)
------------------
* [jsk_topic_tools] add install config directory
* [jsk_topic_tools] Add number of subscribers to diagnostic information
* [jsk_topic_tools/Relay] Add more readable diagnostic including last time it receives input topic
* [jsk_topic_tools/Relay] Add diagnostic information
* [jsk_topic_tools] Update default diagnostic message to be more useful
* Contributors: Yuki Furuta, Ryohei Ueda

2.0.1 (2015-06-28)
------------------
* [jsk_topic_tools] Add DeprecatedRelay nodelet for deprecated topics
* Contributors: Ryohei Ueda

2.0.0 (2015-06-19)
------------------

1.0.72 (2015-06-07)
-------------------
* [jsk_topic_tools] Add global nodehandle
* Contributors: Kentaro Wada

1.0.71 (2015-05-17)
-------------------
* [jsk_topic_tools] Add ~always_subscribe parameter to ConnectionBasedNodelet
  and DiagnosticNodelet to always subscribe input topics
* Contributors: Ryohei Ueda

1.0.70 (2015-05-08)
-------------------
* [jsk_topic_tools/Passthrough] Add ~request service like Snapshot
* Contributors: Ryohei Ueda

1.0.69 (2015-05-05)
-------------------
* [jsk_topic_tools] Shorter test duration for topic_buffer/hztest_chatter_update
* Contributors: Ryohei Ueda

1.0.68 (2015-05-05)
-------------------
* [jsk_topic_tools] Add log_utils.h to print with __PRETY_FUNCTION__
* Contributors: Ryohei Ueda

1.0.67 (2015-05-03)
-------------------
* [jsk_topic_tools] Do not subscribe input if no need in Passthrough nodelet
* [jsk_topic_tools] Remove non-used TransportHint from relay_nodelet
* Contributors: Ryohei Ueda

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------
* [jsk_topic_tools] Publish timestamp from snapshot as it publishes ~output
* [jsk_topic_tools] Add ~stop service to force to stop publishing messages
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------
* [jsk_topic_tools] Add Passthrough nodelet to relay topics during
  specified duration
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------
* [jsk_topic_tools] Add ~latch option to snapshot nodelet
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------
* [jsk_topic_tools] Fix snapshot to publish first message correctly
* [jsk_topic_tools] Add service interface to change output topic of relay node
* anonymous node
* add flatten mode for array type message
* remove space after ,
* add argument exception handler
* add csv exporter for rosbag
* Contributors: Yuki Furuta, Ryohei Ueda

1.0.60 (2015-02-03)
-------------------
* [jsk_topic_tools] add std_srvs

1.0.59 (2015-02-03)
-------------------
* [jsk_topic_tools] Add document about nodelet utility classes
* [jsk_topic_tools] Fix license: WillowGarage -> JSK Lab
* [jsk_topic_tools] Add documentation about color_utils.h
* Remove rosbuild files
* [jsk_topic_tools] Return true in service callback of snapshot nodelet
* [jsk_topci_tools] Fix heatColor function to return std_msgs::ColorRGBA
* [jsk_topic_tools] Add new utility to take snapshot of topic
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [jsk_topic_tools] Indigo test seems to be broken,
  so skip testing on indigo
* [jsk_topic_tools] Do not implement updateDiagnostic
  as pure virtual method
* Reuse isMasterAlive function across scripts which
  want to check master state
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------
* Add function to compute heat color gradient
* Add new script: static_transform_pose_stamped. It looks like tf's
  satatic_transform_publisher but it re-publishes geometry_msgs/PoseStamped.
* Contributors: Ryohei Ueda

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------
* added topic_buffer_periodic_test.launch and added argument to topic_buffer_client/server_sample.launch
* add mutex lock in callback and thread function
* enable to select periodic mode from server param
* enable to select periodic mode from server param
* send request periodic publish from client when rosparam is set
* add update periodically function
* Contributors: Yuki Furuta, Masaki Murooka

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------
* add nodelet to check vital of topic
* Contributors: Ryohei Ueda

1.0.52 (2014-10-23)
-------------------
* Move several utilities for roscpp from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------
* use 300 for default message_num, rostopic hz uses 50000? https://github.com/ros/ros_comm/blob/indigo-devel/tools/rostopic/src/rostopic/__init__.py#L111
* use median instead of average
* Contributors: Kei Okada

1.0.49 (2014-10-13)
-------------------
* Fix location of catkin_package of jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------
* Install executables build as single nodelet
* LightweightThrottle does not subscribe any topics if no need
* fix mutex lock of relay node
* Do not subscribe topics until mux/output is subscribed
* Contributors: Ryohei Ueda

1.0.46 (2014-10-03)
-------------------
* Do not use sleep inside of lightweight_throttle

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------
* Compile transform_merger on catkin
* Use PLUGINLIB_EXPORT_CLASS instead of deprecated PLUGINLIB_DECLARE_CLASS
* Contributors: Ryohei Ueda

1.0.40 (2014-09-19)
-------------------
* Add diagnostic utils from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------
* add new utility function colorCategory20 to jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.36 (2014-09-01)
-------------------
* Add rosparam_utils.cpp: utility functions for ros parameters
* Contributors: Ryohei Ueda

1.0.35 (2014-08-16)
-------------------
* add nodelet.cmake to export utility cmake macro to
  compile nodelet libraries
* Contributors: Ryohei Ueda

1.0.34 (2014-08-14)
-------------------
* add new class: VitalChecker from jsk_pcl_ros
* Contributors: Ryohei Ueda

1.0.33 (2014-07-28)
-------------------
* compile time_acucmulator.cpp on rosbuild environment
* add depend to dynamic_tf_reconfigure
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.32 (2014-07-26)
-------------------
* fix compilation for jsk_topic_tools::TimeAccumulator
* Contributors: Ryohei Ueda

1.0.31 (2014-07-23)
-------------------
* add class TimeAccumulator to measure and accumurate time to jsk_topic_tools
* Contributors: Ryohei Ueda

1.0.30 (2014-07-15)
-------------------
* add tool to check the existence of ros nodes and publish them to diagnostics
* Contributors: Ryohei Ueda

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------
* initialize variable in relay_nodelet
* shutdown subscriber if no need to publish message in relay nodelet
* Merge pull request #466 from garaemon/add-single-executable-for-nodelet
  Add single executables for nodelets of jsk_topic_tools
* add single executable files for each nodelet in jsk_topic_tools
* add test code for block nodelet
* add nodelet to BLOCK topic pipeline according to the number of the subscribers
* add nodelet to relay topic
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.27 (2014-06-10)
-------------------
* add nodelet to relay topic
* Contributors: Ryohei Ueda

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------
* add new nodelet: HzMeasure to measure message rate
* display info in debug mode
* print ignoring tf
* Merge remote-tracking branch 'tarukosu/ignore-specific-transform' into ignore-specific-transform
* add output='screen'
* use joint_states_pruned_buffered instead of _update
* remap /joint_states to /joint_states_pruned_update
* add ignoreing tf config
* add launch file for send joint state and other tf
* prune velocity and effort in joint state
* ignoring tf designated in yaml
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.21 (2014-05-20)
-------------------

1.0.20 (2014-05-09)
-------------------

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------
* change the length of the name field according to the topic now the script subscribes
* print topic name rather than topic index and prettier format
* add test launch file for topic_compare and run it on catkin and rosbuild
* add test script and do not run load_manifest, it's not required
* add topic_compare.py
* Contributors: Ryohei Ueda, Yuki Furuta

1.0.9 (2014-04-12)
------------------
* use ShapeShifter rather than ShapeShifterEvent
* fix for goovy SEGV
  * use ros::Subscriber's pointer
  * use topic_tools::ShapeShiter rather than ShapeShifterEvent
  * not call getPrivateNodeHandle so many times
* Contributors: Ryohei Ueda

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------
* add documentation on nodelet xml
* Contributors: Ryohei Ueda

1.0.6 (2014-04-07)
------------------
* add a sample for mux nodelet and does not use mux nodehandle.
  not using mux NodeHandle is different from original mux in topic_tools.
  now private nodehandle, which is the name of nodelet instance,
  behaves as 'mux' name of mux/topic_tools.
  If you want to use mux_** tools, you just specify nodelet name as mux name.
* implement nodelet version of mux with the same api to topic_tools and no need to specify the
  message type as well as topic_tools/mux
* add rostopic dependency to run test for LightweightThrottle
* update documentation of nodelet xml
* add test code for LightwehgitThrottle
* add a sample launch file for LightwehgitThrottle
* publish data only if any subscriber is
* compile nodelet on rosbuild too
* fixing dependency for nodelet usage
  depends to nodelet on manifest.xml, package.xml and catkin.cmake
* add xml declaration for nodlet plugin
* read update_rate from the parameter ~update_rate
* implement lightweight nodelet throttle
* add lightweight nodelet throttle skelton cpp/header file
* change arg name and node name
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.4 (2014-03-27)
------------------
* move the location of generate_messages and catkin_package to avoid emtpy
  catkin variables problem caused by roseus. it's a hack.
* Contributors: Ryohei Ueda

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: fix typo: dependp -> depend
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add depend tag to jsk_topic_tools/manifest.xml because of previous breaking change of manifest.xml
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: replace .test suffix with .launch in jsk_topic_tools' rosbuild cmake
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add full path to rostest of ros_topic_tools
* Contributors: Ryohei Ueda

1.0.1 (2014-03-07)
------------------
* set all package to 1.0.0
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* fix typo CATKIN-DEPEND -> CATKIN_DEPEND
* add install to catkin.cmake
* (kill_server_and_check_close_wait.py) num=1 is ok for test_close_wait_check?
* add rostest and roscpp_tutorials
* use rosdep instead of depend
* add rostest
* add description in topic buffer sample program
* add buffer client and server for tf
* merge transform message to publish at low rate
* add sample launch files for specific transform
* do not initialize pub_update in use_service mode and restart serviceClient if sc_update.call failed, fixed Issue `#266 <https://github.com/jsk-ros-pkg/jsk_common/issues/266>`_
* rename to test_topic_buffer_close_wait.launch and add kill_server_and_check_close_wait.py
* add test launch for CLOSE_WAIT problem
* fixing output of ROS_INFO
* supporting topicized /update and parameterized /list
* fix test code chatter_update only publish every 10 min
* update topic_buffer_server/cliet, client automatically calls /update service to get latest information on server side ,see Issue `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_
* support update_rate param to configure how often client calls /update, see issue `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_
* client to call update to get current information on publish rate
* add rosbuild_add_rostest
* fix output message
* fix problem reported on `#260 <https://github.com/jsk-ros-pkg/jsk_common/issues/260>`_, add test code
* add more verbose message
* add sample launch file using topic_buffer
* update for treating multiple tf
* wait until service is available
* add specific transform publisher and subscriber
* add fixed_rate and latched parameter
* make catkin to work jsk_topic_tools
* add update service in topic_buffer_server
* fix xml: catkinize jsk_topic_tools
* fix broken xml: catkinize jsk_topic_tools
* fix broken xml: catkinize jsk_topic_tools
* catkinize jsk_topic_tools
* add jsk_topic_tools
* Contributors: Ryohei Ueda, Kei Okada, youhei, Yusuke Furuta
