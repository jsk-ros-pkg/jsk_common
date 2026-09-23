#ifndef JSK_ROS1_ROS2_COMPAT_ROS_VERSION_H_
#define JSK_ROS1_ROS2_COMPAT_ROS_VERSION_H_

// ROS1/ROS2 detection: self-detect from whether <ros/ros.h> (ROS1's
// roscpp) is includable at all, rather than relying on CMakeLists.txt
// to pass -DROS2_BUILD -- see posedetection_msgs/feature0d_to_image.h
// (jsk_common_msgs) for the reference pattern. <ros/ros.h> (via
// ros/common.h) #defines ROS_VERSION_MAJOR to 1; when unavailable the
// macro is simply undefined, and an undefined macro in a preprocessor
// #if evaluates to 0, so `#if ROS_VERSION_MAJOR != 1` reliably means
// "ROS2" without needing a macro of our own.
//
// This header only pulls in <ros/ros.h> (ROS1) / nothing (ROS2) -- no
// message types, no ConstPtr macro -- so it's always safe to include
// first, before any other headers (including a package's own
// generated message headers). Contrast with const_ptr.h, which must
// be included *after* those.
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif

#if ROS_VERSION_MAJOR != 1
#include <rclcpp/rclcpp.hpp>
#endif

#endif  // JSK_ROS1_ROS2_COMPAT_ROS_VERSION_H_
