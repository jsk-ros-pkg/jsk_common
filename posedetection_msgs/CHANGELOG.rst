^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package posedetection_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
