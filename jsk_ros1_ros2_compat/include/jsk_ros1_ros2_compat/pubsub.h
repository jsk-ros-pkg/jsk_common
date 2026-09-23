#ifndef JSK_ROS1_ROS2_COMPAT_PUBSUB_H_
#define JSK_ROS1_ROS2_COMPAT_PUBSUB_H_

#include <jsk_ros1_ros2_compat/ros_version.h>

// Publisher/Subscriber compatibility: ros::Publisher/Subscriber are
// type-erased (one concrete type for any message), while
// rclcpp::Publisher<T>/Subscription<T> are templated per message type
// -- so member declarations use these alias templates (parameterized
// on the ROS1-spelled message type, e.g.
// ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::Image>, which resolves to
// the ROS2 msg:: type when built for ROS2) instead of repeating the
// whole member list once per ROS version. `publishMsg()`/`topicName()`
// similarly paper over `pub.publish(x)`/`pub.getTopic()` (ROS1, value
// member) vs `pub->publish(x)`/`pub->get_topic_name()` (ROS2, shared_ptr
// member).
namespace ROS1_ROS2_COMPAT
{
#if ROS_VERSION_MAJOR != 1
template<typename T> using RosPublisher = typename rclcpp::Publisher<T>::SharedPtr;
template<typename T> using RosSubscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T> using RosServiceServer = typename rclcpp::Service<T>::SharedPtr;
template<typename PubT, typename MsgT>
inline void publishMsg(PubT& pub, const MsgT& msg) { pub->publish(msg); }
template<typename PubT>
inline std::string topicName(PubT& pub) { return pub->get_topic_name(); }
// ROS1 accepts a bare relative topic name (e.g. "input/image") from a
// private ("~") NodeHandle; ROS2 requires the explicit "~/" prefix for
// the same private-topic meaning.
inline std::string privateName(const std::string& name) { return "~/" + name; }
#else
template<typename T> using RosPublisher = ros::Publisher;
template<typename T> using RosSubscriber = ros::Subscriber;
template<typename T> using RosServiceServer = ros::ServiceServer;
template<typename PubT, typename MsgT>
inline void publishMsg(PubT& pub, const MsgT& msg) { pub.publish(msg); }
template<typename PubT>
inline std::string topicName(PubT& pub) { return pub.getTopic(); }
inline std::string privateName(const std::string& name) { return name; }
#endif

// Subscription creation: ros::NodeHandle::subscribe(topic, queue_size,
// callback) (ROS1) accepts any boost::function-compatible callable, so
// a std::bind(&Class::method, this, std::placeholders::_1) result
// works for both ROS1's nh->subscribe() and ROS2's
// node->create_subscription<T>() -- letting createSubscriber() paper
// over the two calls (plus the private-topic "~/" prefix difference,
// via privateName() above) with the same call site for both versions.
// `nh` is a pointer/shared_ptr to the owning NodeHandle/Node in both
// cases.
template<typename T, typename NhPtrT, typename CallbackT>
inline RosSubscriber<T> createSubscriber(NhPtrT& nh, const std::string& topic, int queue_size, CallbackT callback)
{
#if ROS_VERSION_MAJOR != 1
  return nh->template create_subscription<T>(privateName(topic), queue_size, callback);
#else
  // Explicit boost::function construction: template argument deduction
  // for NodeHandle::subscribe()'s boost::function overload can't infer
  // M from a bare std::bind(...) result (no user-defined conversions
  // considered during deduction), so it must already have this exact
  // type by the time it's passed in.
  boost::function<void(const boost::shared_ptr<T const>&)> fn(callback);
  return nh->subscribe(privateName(topic), queue_size, fn);
#endif
}
}  // namespace ROS1_ROS2_COMPAT

#endif  // JSK_ROS1_ROS2_COMPAT_PUBSUB_H_
