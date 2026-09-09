#ifndef JSK_ROS1_ROS2_COMPAT_LOGGING_H_
#define JSK_ROS1_ROS2_COMPAT_LOGGING_H_

#include <jsk_ros1_ros2_compat/ros_version.h>

namespace ROS1_ROS2_COMPAT
{
#if ROS_VERSION_MAJOR != 1
  // Set once by each executable's main()/initialize(), before any
  // ROS_INFO()-style macro or ros::param::param() call below -- there
  // is only ever one node per process for the executables using this
  // header. `inline` avoids an ODR violation from this header being
  // included by multiple translation units.
  inline rclcpp::Node::SharedPtr g_node;
#define ROS_DEBUG(...) RCLCPP_DEBUG(ROS1_ROS2_COMPAT::g_node->get_logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(ROS1_ROS2_COMPAT::g_node->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(ROS1_ROS2_COMPAT::g_node->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ROS1_ROS2_COMPAT::g_node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(ROS1_ROS2_COMPAT::g_node->get_logger(), msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(ROS1_ROS2_COMPAT::g_node->get_logger(), msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(ROS1_ROS2_COMPAT::g_node->get_logger(), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(ROS1_ROS2_COMPAT::g_node->get_logger(), msg)
#endif
}

// ros::param::param<T>(name, value, default) is ROS1's real API (reads
// a private "~name" param, or the given default); reopening it here
// under ROS2 -- backed by the same g_node used for logging above --
// lets "~foo" ros::param::param<T>(...) call sites stay completely
// unchanged from the ROS1 original instead of introducing a new compat
// function name.
#if ROS_VERSION_MAJOR != 1
namespace ros
{
  namespace param
  {
    template<typename T>
    void param(const std::string &name, T &value, const T &default_value)
    {
      std::string stripped_name = (!name.empty() && name[0] == '~') ? name.substr(1) : name;
      value = ROS1_ROS2_COMPAT::g_node->declare_parameter<T>(stripped_name, default_value);
    }
  }
}
#endif

#endif  // JSK_ROS1_ROS2_COMPAT_LOGGING_H_
