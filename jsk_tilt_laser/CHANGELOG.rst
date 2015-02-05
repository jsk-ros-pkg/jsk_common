^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tilt_laser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
