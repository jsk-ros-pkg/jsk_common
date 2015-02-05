^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package collada_urdf_jsk_patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------
* add patch for removing old gazebo settings
* Contributors: Yohei Kakiuchi

1.0.55 (2014-12-09)
-------------------
* fix compiling jsk_ros_patch for indigo
* Contributors: Yohei Kakiuchi

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------
* Fix a logic error in collada_jsk_patch
  This fixes a regression caused by 9846892b8ec1c1b3e655015298cd9a8e17b155e7
* Contributors: Scott K Logan

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* disable collada_urdf_jsk_patch for indigo
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

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------

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
* install collada_to_urdf

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
* copy collada_to_urdf binary to devel directory
* Contributors: Masaki Murooka

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------
* disable ssl setting for download robot_model
* Contributors: Kei Okada

1.0.16 (2014-04-19)
-------------------
* add depends to collada_parser, collada_urdf, urdf and kdl_parser
* Contributors: Kei Okada

1.0.15 (2014-04-19)
-------------------
* add depend to class_loader, pluginlib, rostest
* Contributors: Kei Okada

1.0.14 (2014-04-19)
-------------------
* add missing deps(mk,git,..) to collada_urdf_jsk_patch
* Contributors: Kei Okada

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------
* update collada_urdf to use assimp_devel on hydro-devel
* Contributors: Kei Okada

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.6 (2014-04-07)
------------------
* fix to work with hydro (which uses same setup with groovy)
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* use rosdep instead of depend package
* set target name as urdf_to_collada
* copy urdf_to_collada bin file to CATKIN_PACKAGE_BIN_DESTINATION
* add caktin buildtool_depend and find_package, catkin_package
* catkinize collada_urdf_jsk_patch
* change robot_model repository from kforge to github on fuerte, [`#227 <https://github.com/jsk-ros-pkg/jsk_common/issues/227>`_]
* pull request merged ( https://github.com/ros/robot_model/commit/2eaf5c9166ebd50cbc14cf807d3d09b0597ee045 )
* add collada_cmake.patch for compiling on groovy
* add set_url_name_groovy.patch for compiling on groovy
* revert set_url_name.patch for compiling on fuerte
* add temporary patch for using multiple visual, it pull requested at https://github.com/ros/robot_model/pull/20
* update for using repository in github
* fix for assimp3 which aiScene is hiden
* fix for groovy
* download collada-dom-2.2.zip from jsk-ros-pkg, pr2.willowgarage.com has stopped?
* robot_model repository moved to github, temporary using latest hg repository
* use collada-dom 2.4 for groovy
* fix HG_ROS_PACKAGE_PATH -> ROS_PACKAGE_PATH
* fix for groovy
* use PLATFORM_FLOAT64 for daeFloat, collada-fom for groovy uses -DCOLLADA_DOM_DAEFLOAT_IS64, update pr2.l to use double precision value
* fix segfault on groovy problem https://github.com/ros/robot_model/issues/4
* fix to compile on groovy?
* add ColladaDOM150 namespace
* fix for groovy
* fix to compile on groovy
* fix to compile on groovy
* fix to compile on groovy
* use http instead of https to avoid certificate verify failure
* add set_url_name patch
* clean up and force remove urdf_to_collada when make clean
* fix Makefile error in collada_urdf_jsk_patch
* fix for hg https://code.ros.org/trac/ros/ticket/3748
* use robot_model version from rosversion
* update to electric
* fix download robot_model-1.5.1_hg
* rename colada_urdf_hg to robot_model-1.5.1_hg
* fix Makefile syntax error
* fix to work with electric : hg_checkout.mk is changed
* make clean to remove rosdep.yaml files
* set HG_REVISION not HG_BRANGE
* update to hg repository
* update tags cturtle->robot_model-1.4.0
* add debian info to rosdep.yaml
* collada format uses degree for upper and lower limits
* add radlimit patch to output limit in radius
* get geometry data from geometry.get instead of urdf_link->visual for SPHERE,BOX,CYLINDER
* add more error checking to avoid segfault
* update not to run rosmake in Makefile
* collada_urdf_jsk_patch does not depends on collada_urdf
* add jsk patch for collada_urdf, that support material, cube, cylinder, sphere
* Contributors: Ryohei Ueda, Kei Okada, youhei
