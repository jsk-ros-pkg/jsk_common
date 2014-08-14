^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Kei Okada, furuta, k-okada, ueda, youhei
