^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package collada_urdf_jsk_patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

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
* Contributors: Kei Okada, k-okada, ueda, youhei
