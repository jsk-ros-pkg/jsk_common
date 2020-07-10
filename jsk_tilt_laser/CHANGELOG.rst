^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tilt_laser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.11 (2020-07-10)
-------------------
* Fix for noetic build (`#1648 <https://github.com/jsk-ros-pkg/jsk_common/issues/1648>`_)

  * fix for python3, except, print ....
  * upgrade package.xml to format=3

* Contributors: Kei Okada

2.2.10 (2018-11-03)
-------------------

2.2.9 (2018-11-02)
------------------

2.2.8 (2018-11-01)
------------------
* add tilt laser pipeline for jsk_tilt_laser (`#1605 <https://github.com/jsk-ros-pkg/jsk_common/issues/1605>`_)
  * [jsk_tilt_laser] add new version of launch script for tilt laser
  * [jsk_tilt_laser] change joint name of tilt_laser

* Add comment about installation (`#1604 <https://github.com/jsk-ros-pkg/jsk_common/issues/1604>`_)
* update jsk_travis to 0.4.38 add lunar and melodic (`#1594 <https://github.com/jsk-ros-pkg/jsk_common/issues/1594>`_)
  * use find_package(Eigen3) instead of find_package(Eigen)
  * use find_package(multisense_lib QUIET) to check multisense_lib

* Contributors: Kei Okada, Yohei Kakiuchi, Yuto Uchimi

2.2.7 (2018-06-27)
------------------
* add missing dirs into install command (`#1583 <https://github.com/jsk-ros-pkg/jsk_common/issues/1583>`_)
  * add missing dirs into install command
* Contributors: Yasuhiro Ishiguro

2.2.6 (2018-01-05)
------------------

2.2.5 (2017-06-19)
------------------

2.2.4 (2017-06-14)
------------------
* [multisense_laser_pipeline] fix from inside the JAXON (`#1528 <https://github.com/jsk-ros-pkg/jsk_common/pull/1528>`_)
* Contributors: Yohei Kakiuchi

2.2.3 (2017-03-23)
------------------

2.2.2 (2016-12-30)
------------------

2.2.1 (2016-12-13)
------------------

2.2.0 (2016-10-28)
------------------

2.1.2 (2016-09-14)
------------------

2.1.1 (2016-09-07)
------------------

2.1.0 (2016-09-06)
------------------

2.0.17 (2016-07-21)
-------------------

2.0.16 (2016-06-19)
-------------------

2.0.15 (2016-06-13)
-------------------

2.0.14 (2016-05-14)
-------------------

2.0.13 (2016-04-29)
-------------------

2.0.12 (2016-04-18)
-------------------

2.0.11 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* CATKIN_INCLUDE_DIRS -> catkin_INCLUDE_DIRS
* Contributors: Kei Okada, Kentaro Wada

2.0.10 (2016-02-13)
-------------------
* Support jsk_tilt_laser and jsk_topic_tools on OS X
* [jsk_tilt_laser] Publish snapshot laser assembled cloud
* [jsk_tilt_laser] Remap tf topics only for pcl_ros nodelets
* [jsk_tilt_laser] Optimize /joint_states and /tf around multisense.
  1. Add /multisense_local/joint_states topic for the nodes which
  are only interested in multisense joint angles
  2. Remap /tf to /tf_null in multisense_laser to avoid tf listening
  from pcl/PassThrough
* Contributors: Ryohei Ueda

2.0.9 (2015-12-14)
------------------

2.0.8 (2015-12-07)
------------------

2.0.7 (2015-12-05)
------------------
* jsk_tilt_laser/README.md: fix section/subsection
* Contributors: Kei Okada

2.0.6 (2015-12-02)
------------------

2.0.5 (2015-11-30)
------------------

2.0.4 (2015-11-25)
------------------
* [jsk_tilt_laser] Remove image_rect pipeline temporary because of high load. It is needed to be tuned.
* [jsk_tilt_laser] Add image_rect multi_resolution pipeline
* [jsk_tilt_laser] Add NODELET_INDEX options to use same manager with different rosparam by nodelets_i
* [jsk_tilt_laser] Separate left camera stereo image pipeline
* [jsk_tilt_laser] Include for multi resolution image for right camera
* [jsk_tilt_laser] Add viewer to check multisense local topics
* [jsk_tilt_laser] Specify twist_frame_id of multisense laser to publish velocity
* [jsk_tilt_laser] Add run_distance_filters argument to disable filtering based on distance
* Contributors: Ryohei Ueda, hrpuser, Iori Kumagai

2.0.3 (2015-07-24)
------------------
* [jsk_topic_tools] Rewrite multisense_remote.launch with standalone_complex_nodelet
* [jsk_tilt_laser] Rewrite multisense_laser_pipeline.launch with standalone_complexed_nodelet
* [jsk_tilt_laser] Add assemble_cloud argument to customize laser pipeline
* [jsk_tilt_laser] Fix multisense pointcloud topic for local mode
* Contributors: Ryohei Ueda

2.0.2 (2015-07-07)
------------------

2.0.1 (2015-06-28)
------------------
* [jsk_tilt_laser] Add multisense_killer to kill multisense streaming if
  host PC is down
* Contributors: Ryohei Ueda

2.0.0 (2015-06-19)
------------------
* [jsk_tilt_laser] Do not use laser_assembler in order to reduce CPU load
* Contributors: Ryohei Ueda

1.0.72 (2015-06-07)
-------------------

1.0.71 (2015-05-17)
-------------------
* [jsk_tilt_laser] Update threshold to remove laser noise of multisense
* [jsk_tilt_laser] Increase minimum intensity to use on multisense
* [jsk_tilt_laser/multisense_laser_pipeline.launch] Add xyz filter before
  downsample pointcloud in order to avoid overflow of indices
* Contributors: Ryohei Ueda

1.0.70 (2015-05-08)
-------------------
* [jsk_tilt_laser] More detailed laser pointcloud for precise perception
* Contributors: Ryohei Ueda

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------

1.0.67 (2015-05-03)
-------------------
* [jsk_tilt_laser] Use resized and compressed images to reconstruct
  multisense pointcloud
* [jsk_tilt_laser] Do not use spindle_half model in order to decrease risk of
  dropping of tilt laser scans
* [jsk_tilt_laser] Add arguments for resized images in multisense.launch
* [jsk_tilt_laser] Increase queue size of point_xyz and point_xyzrgb in multisense_remote.launch
* [jsk_tilt_laser] Downsample pointcloud in default
* [jsk_tilt_laser] Fix indent and typo
* [jsk_network_tools] Load laser_pipeline.launch from multisense_remote.launch
* [jsk_tilt_laser] Fix indent
* [jsk_tilt_laser] Use compressed rgb image to colorize pointcloud and
  separate laser pipeline into multisense_laser_pipeline.launch
* [jsk_tilt_laser] Relay multisense_local/left/camera_info to
  multisense/left/camera_info in remote machine
* Merge remote-tracking branch 'refs/remotes/origin/master' into multisense-local
  Conflicts:
  jsk_tilt_laser/launch/multisense.launch
* [jsk_tilt_laser] Add local argument to multisense.launch and add multisense_remote.launch
  to separatly run multisense driver
* [jsk_tilt_laser] Add options to run multisense local mode
* Contributors: Ryohei Ueda

1.0.66 (2015-04-03)
-------------------
* [jsk_tilt_laser] Add fixed_frame_id argument to multisense.launch
* Contributors: Ryohei Ueda

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------
* [jsk_tilt_laser] Support multisense sensors without 'multisense/' prefix
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------
* [jsk_tilt_laser, jsk_data] Add multisense_play.launch to play multisene bag file
* [jsk_tilt_laser] Do not use filters in laser assmble node
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------
* [jsk_tilt_laser] Add another argument to disable robot_state_publisher and
  robot_description perfectly
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------
* [jsk_tilt_laser] Add sensor_tf_prefix and not_use_sensor_tf_prefix for the
  robots which has '/left/image_rect_color' sensor frame instead of
  '/multisense/left/image_rect_clor'
* Contributors: Ryohei Ueda

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* [jsk_tilt_laser] Update multisense.launch according to the latest update
  1) use multisense.launch, it support launch_robot_state_publisher argument
  2) fix name to change speed of spindle laser
* Remove rosbuild files
* [jsk_tilt_laser] Add ~overwrap_angle parameter to multisense.launch
* [jsk_tilt_laser] Add scan_to_cloud_chain to multisense.launch to get
  one-scan pointcloud. We use ~high_fidelity=true in order to avoid
  laser_geometry's bug to produce large sphere pointcloud
* Merge pull request `#691 <https://github.com/jsk-ros-pkg/jsk_common/issues/691>`_ from garaemon/laser-filter
  [jsk_tilt_laser] Add laser_filters to multisense
* [jsk_tilt_laser] Add laser_filters to multisense
* update multisense launch for using with real robot
* Contributors: Ryohei Ueda, Yohei Kakiuchi

1.0.58 (2015-01-07)
-------------------
* [jsk_tilt_laser] Use jsk_pcl_ros/TiltLaserListener rather than
  jsk_tilt_laser's snapshotter.
* [jsk_tilt_laser] Add use_robot_description argument to multsense.launch and removed robot_description private param in ros_driver, which is seemed to be unused in multisense_ros/src
* Add document about dynamixel permission on jsk_tilt_laser
* add downsampled points to multisense.launch in jsk_tilt_laser
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------
* Added parameter configuration for fps and spindle_speed of multisense and fixed urdf name
* Contributors: Ryo Terasawa

1.0.54 (2014-11-15)
-------------------
* Added tilt_laser.urdf.xacro to mount on a urdf of TurtleBot.
* Contributors: Tanaka Yoshimaru

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------
* Add dynamic_reconfigure and sensor_msgs to jsk_tilt_laser depdendency
* Add missing deps to jsk_tilt_laser
* Contributors: Ryohei Ueda, Scott K Logan

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* add cmake_modules for indigo compile
* Contributors: Kei Okada

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------

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

1.0.40 (2014-09-19)
-------------------
* Add spin_laser_assmbler to build pointcloud from spining laser and add
  launch and config files for multisense SL.
* Contributors: Ryohei Ueda

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------
* update CHANGELOG.rst
* Add ~tilt_joint_name parameter to tilt_laser_assembler.py to specify the joint name
  of tilt laser
* Change scan time according to change of joint state
* Contributors: Ryohei Ueda

1.0.37 (2014-09-08)
-------------------
* commonize tilt_laser_assembler
* added codes to controll tilt_speed with dynamixel_reconfigure
* Fix: rospy.debug -> rospy.logdebug
* add jsk_tilt_laser
* Contributors: Yuki Furuta, Ryohei Ueda, Yohei Kakiuchi

1.0.36 (2014-09-01)
-------------------

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

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------

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

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
