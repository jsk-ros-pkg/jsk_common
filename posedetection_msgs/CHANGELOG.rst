^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package posedetection_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.4 (2014-03-27)
------------------
* posedetection_msgs: add message_generation to package.xml

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* use USE_ROSBUILD for catkin/rosbuild environment
* remove debug code
* use ROS_Distributions instead of ROS_DISTRO for electric
* comment out : add catkin.cmake
* add depends to roscpp
* update to opencv2
* move depend opencv2 to jsk_common since posedetection_msgs is under jsk_common stack
* add rosdep2 for electric, will fixed in opencv2.4. https://code.ros.org/trac/ros-pkg/ticket/5437
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* Contributors: Kei Okada, k-okada, rosen
