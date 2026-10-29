^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_ros1_ros2_compat
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-10-29)
------------------
* Merge pull request `#1826 <https://github.com/jsk-ros-pkg/jsk_common/issues/1826>`_ from k-okada/ros2-support
  Add ROS2 support to jsk_ros1_ros2_compat, dynamic_tf_publisher, jsk_topic_tools, jsk_tools, audio_video_recorder, jsk_data, image_view2, jsk_rosbag_tools
* [jsk_ros1_ros2_compat] new package: shared ROS1/ROS2 compat helpers
  Consolidates the ROS1/ROS2 compatibility boilerplate needed by this
  branch's ported packages, so each package doesn't maintain its own
  near-duplicate copy:
  - include/jsk_ros1_ros2_compat/compat.h: the __has_include(<ros/ros.h>)
  self-detection block, the ROS1_ROS2_COMPAT namespace (RosTime/
  RosDuration/rosTimeNow()/RosPublisher<T>/RosSubscriber<T>/
  RosServiceServer<T>/publishMsg()/topicName(), backed by a single
  g_node set once per process), the ROS_INFO/ROS_ERROR/ROS_WARN/
  ROS_DEBUG (+_STREAM) logging-macro shim, and ros::param::param<T>()
  reopened under ROS2.
  - src/jsk_ros1_ros2_compat/rospy_rclpy_compat.py: an rospy/rclpy compat
  shim (get_param, log*, create_publisher/subscription/service, ...)
  for packages using rclpy under ROS2.
  - src/jsk_ros1_ros2_compat/log_utils.py: jsk_log*()/*_throttle()
  logging helpers on top of rospy_rclpy_compat's node-registration
  pattern.
  This package has no dependencies on any other jsk_common package, so
  depending on it never risks a dependency cycle. Consuming packages are
  migrated to it in the following commits.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
* Contributors: Kei Okada
