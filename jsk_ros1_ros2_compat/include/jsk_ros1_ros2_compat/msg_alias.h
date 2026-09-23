#ifndef JSK_ROS1_ROS2_COMPAT_MSG_ALIAS_H_
#define JSK_ROS1_ROS2_COMPAT_MSG_ALIAS_H_

#include <jsk_ros1_ros2_compat/ros_version.h>

// Message-type compatibility: ROS2 generated message types live in a
// `msg::` sub-namespace (e.g. geometry_msgs::msg::PoseStamped) unlike
// ROS1's flat geometry_msgs::PoseStamped.
// JSK_ROS1_ROS2_COMPAT_MSG_ALIAS reopens the ROS1-style flat name as a
// `using` alias to the ROS2 one (a no-op under ROS1, where the flat
// name is already the real type), so message-type references at call
// sites don't need to change between versions -- e.g.
// `JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, PoseStamped)` lets
// `geometry_msgs::PoseStamped` keep working under both.
//
// Safe to include (and invoke) anywhere, in any order relative to
// message headers, since it only expands where explicitly invoked --
// contrast with const_ptr.h, which must come after them.
#if ROS_VERSION_MAJOR != 1
#define JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(pkg, type) \
  namespace pkg { using type = pkg::msg::type; }
// Same idea, for services: ROS2 puts them in a `srv::` sub-namespace
// (e.g. std_srvs::srv::Empty) instead of msg::.
#define JSK_ROS1_ROS2_COMPAT_SRV_ALIAS(pkg, type) \
  namespace pkg { using type = pkg::srv::type; }
// ROS1 message generation provides both `pkg::Type::ConstPtr` (a
// member typedef) *and* a flat `pkg::TypeConstPtr` alias; call sites
// written against the latter (e.g. `sensor_msgs::ImageConstPtr`) need
// this under ROS2 only, since const_ptr.h's `ConstPtr`->`ConstSharedPtr`
// macro does not cover the flat spelling. A no-op under ROS1, where
// message generation already provides the flat name natively.
#define JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(pkg, type) \
  namespace pkg { using type##ConstPtr = pkg::msg::type::ConstSharedPtr; }
// Same idea for a service's request/response: ROS1 gencpp provides
// flat `pkg::TypeRequest`/`pkg::TypeResponse` aliases (alongside the
// nested `pkg::Type::Request`/`::Response`); ROS2 rosidl only
// generates the nested spelling, so call sites written against the
// flat one need this under ROS2 only.
#define JSK_ROS1_ROS2_COMPAT_SRV_REQ_RES_ALIAS(pkg, type) \
  namespace pkg { \
    using type##Request = pkg::srv::type::Request; \
    using type##Response = pkg::srv::type::Response; \
  }
#else
#define JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(pkg, type)
#define JSK_ROS1_ROS2_COMPAT_SRV_ALIAS(pkg, type)
#define JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(pkg, type)
#define JSK_ROS1_ROS2_COMPAT_SRV_REQ_RES_ALIAS(pkg, type)
#endif

#endif  // JSK_ROS1_ROS2_COMPAT_MSG_ALIAS_H_
