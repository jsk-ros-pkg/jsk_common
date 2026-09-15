#ifndef JSK_ROS1_ROS2_COMPAT_CONST_PTR_H_
#define JSK_ROS1_ROS2_COMPAT_CONST_PTR_H_

#include <jsk_ros1_ros2_compat/ros_version.h>

// `ConstPtr` as a bare token (not part of a larger identifier like
// `ImageConstPtr`) only ever appears as `<msg-or-srv-type>::ConstPtr`,
// so redefining it package-wide to ROS2's `::ConstSharedPtr` is safe
// -- for call sites. It is NOT safe while a ROS2 message header itself
// is being parsed: rosidl-generated message classes define their own
// `ConstPtr` member (a deprecated alias of `ConstSharedPtr`, for ROS1-
// migration convenience), and this #define would rewrite *that*
// declaration into a duplicate `ConstSharedPtr`, a hard compile error.
//
// So: include this header LAST, after every message/service header
// your file needs (including transitively, e.g. via <rclcpp/rclcpp.hpp>
// or other jsk_ros1_ros2_compat headers) -- never at the top of a file.
#if ROS_VERSION_MAJOR != 1
#define ConstPtr ConstSharedPtr
#endif

#endif  // JSK_ROS1_ROS2_COMPAT_CONST_PTR_H_
