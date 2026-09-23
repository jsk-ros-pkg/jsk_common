#ifndef JSK_ROS1_ROS2_COMPAT_COMPAT_H_
#define JSK_ROS1_ROS2_COMPAT_COMPAT_H_

// Convenience aggregate of everything in jsk_ros1_ros2_compat that is
// safe to include anywhere, in any order relative to your own message
// headers: ROS1/ROS2 detection, the JSK_ROS1_ROS2_COMPAT_MSG_ALIAS()/
// JSK_ROS1_ROS2_COMPAT_SRV_ALIAS() macros, the ROS_INFO/ROS_ERROR/...
// logging macros + ros::param::param() shim, RosTime/RosDuration/
// rosTimeNow()/..., and RosPublisher<T>/RosSubscriber<T>/publishMsg()/....
//
// NOT included here: const_ptr.h (the `#define ConstPtr ConstSharedPtr`
// shim). That one is only safe to include *after* every message/service
// header your file needs -- ROS2's generated message classes have their
// own `ConstPtr` member (a deprecated alias for ConstSharedPtr), and
// this #define would corrupt it if active while such a header is being
// parsed. Include jsk_ros1_ros2_compat/const_ptr.h explicitly, as the
// last include in your file, if you need it.
#include <jsk_ros1_ros2_compat/ros_version.h>
#include <jsk_ros1_ros2_compat/msg_alias.h>
#include <jsk_ros1_ros2_compat/logging.h>
#include <jsk_ros1_ros2_compat/time.h>
#include <jsk_ros1_ros2_compat/pubsub.h>

#endif  // JSK_ROS1_ROS2_COMPAT_COMPAT_H_
