^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_view2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* publish the mouse position to movepoint topic during mouse move event
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

1.0.21 (2014-05-20)
-------------------
* does not check 0.5sec test if the image_view2 is in series mode.
* not use ros::Rate's sleep, use cvWaitKey to captuere
  keys to be pressed
* Contributors: Ryohei Ueda

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
* image_View2:add message_generation, message_runtime to package.xml
* in order to avoid empty catkin_LIBRARIES problem, call generate_messaegs after target_link_libraries
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* Contributors: Kei Okada, Ryohei Ueda

1.0.2 (2014-03-12)
------------------
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add dependency image_view2 to image_view
* fix image_view2 dependency for rosbuild environment
* Contributors: Ryohei Ueda, nozawa

1.0.1 (2014-03-07)
------------------
* added CIRCLE3D type marker sample
* add CIRCLE3D type marker
* Contributors: HiroyukiMikita, Kei Okada

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* install image_view2
* use rosdep instead of depend package
* add find_package PCL for catkin
* supporting series selection in addition to rectangle selection
* use image_transport parameter, it is the same as image_view
* change for updating drawing while not image comming
* adding dependency to generation_message
* add show_info parameter to display curret frame rate, see Issue 247
* catkinize image_view2
* fix all the indent and add the function to fill in the polygon
* add function to draw in the circle
* new parameter: tf_timeout
* support to set the width of a line
* add ~resize_scale_x, ~resize_scale_y parameters for using resized image
* add subscribing point click
* add points_rectangle_extractor.cpp
* changed text msg visualizationo, bigger textsize and color
* add 3d strip/list/polygon/points/text  `#850 <https://github.com/jsk-ros-pkg/jsk_common/issues/850>`_
* fix typo
* add use_window param
* fix for fuerte
* fix deprecated functions
* update comment for TEXT
* use scale for size of the font
* add text example
* fix putText
* check lastCommonTime
* add comments
* added a flag for action==REMOVE&&id==-1, for clear all the markers
* namespace std is needed in image_view2.cpp
* add blurry mode
* set points size to 10
* fix out_msg.encoding from TYPE_32FC1 to bgr8
* update deprecated funcitons to current function api for cam_model
* change fond and use ROS_DEBUG to display tf exception
* send TF exception error at fist 5 times
* changed debug messages for markers from ROS_INFO to ROS_DEBUG
* update to new roseus msg format
* remove deprecated codes
* update to support bayer image and move to cv2
* draw selecting rectangle every time
* add TEXT type marker, only simple outputs yet
* enable ADD/REMOVE action, lifetime, marker colors partially
* change marker_sub buffer from 1 to 10
* remove /reset_time
* publish screenpoint and screenrectangle on namespace + imagetopic_name
* add example to see gripper_tool_frame in image_view2
* remove unused function cmvision-cb
* back to previous version, which is not using subscribeCamera, becouse of slow connection of pr2-network
* rewrite using subscribeCamera
* add image_view2/
* Contributors: Kei Okada, chen, k-okada, kazuto, manabu, mikita, ueda, youhei
