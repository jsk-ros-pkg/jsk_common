#ifndef JSK_ROS1_ROS2_COMPAT_TIME_H_
#define JSK_ROS1_ROS2_COMPAT_TIME_H_

#include <jsk_ros1_ros2_compat/logging.h>  // for g_node, used by rosTimeNow()

// Time/Duration compatibility layer (keeps ros::Time-style call sites
// unchanged rather than touching every one of them):
//  - RosTime/RosDuration alias the per-version type.
//  - `.toSec()` is a macro to `.seconds()` under ROS2 -- rclcpp::Time/
//    Duration's semantic equivalent -- so existing `.toSec()` call
//    sites need no change (the bare `toSec` token, always used as a
//    method call, is safe to redefine package-wide). Macros are
//    preprocessor tokens, not C++ symbols, so they cannot be namespaced.
//  - rosTimeNow() replaces `ros::Time::now()` call sites (ROS2 has no
//    such static/global accessor; it needs a Clock from a node).
namespace ROS1_ROS2_COMPAT
{
#if ROS_VERSION_MAJOR != 1
using RosTime = rclcpp::Time;
using RosDuration = rclcpp::Duration;
#define toSec seconds
inline RosTime rosTimeNow() { return ROS1_ROS2_COMPAT::g_node->get_clock()->now(); }
// message Header.stamp fields are builtin_interfaces::msg::Time under
// ROS2 (needs RosTime::to_msg()), but plain ros::Time under ROS1
// (directly assignable) -- this bridges `header.stamp = stampMsg(t)`.
inline builtin_interfaces::msg::Time stampMsg(const RosTime& t) { return t; }
// ros::Duration(double) is a constructor; rclcpp::Duration has no such
// constructor, only the static factory from_seconds(double).
inline RosDuration durationFromSec(double seconds) { return rclcpp::Duration::from_seconds(seconds); }
// a message `duration` field is builtin_interfaces::msg::Duration under
// ROS2 (a plain struct, no .toSec()/.seconds() method of its own) but
// plain ros::Duration under ROS1 (already has .toSec()) -- this bridges
// `durationFromMsg(msg_field).toSec()`.
inline RosDuration durationFromMsg(const builtin_interfaces::msg::Duration& d) {
  return RosDuration(d.sec, d.nanosec);
}
#else
using RosTime = ros::Time;
using RosDuration = ros::Duration;
inline RosTime rosTimeNow() { return ros::Time::now(); }
inline RosTime stampMsg(const RosTime& t) { return t; }
inline RosDuration durationFromSec(double seconds) { return ros::Duration(seconds); }
inline RosDuration durationFromMsg(const ros::Duration& d) { return d; }
#endif
}  // namespace ROS1_ROS2_COMPAT

#endif  // JSK_ROS1_ROS2_COMPAT_TIME_H_
