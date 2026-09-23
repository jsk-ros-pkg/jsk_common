#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

#include <jsk_ros1_ros2_compat/compat.h>

// tf/tfMessage (ROS1) is a deprecated alias for tf2_msgs/TFMessage (same
// fields/md5sum); tf2_msgs::msg::TFMessage is the ROS2 spelling of the
// same wire type, reopened here so `tf2_msgs::TFMessage` (unqualified by
// `msg::`) resolves under both versions -- letting transformCallback's
// signature below be shared verbatim by ROS1 and ROS2.
#if ROS_VERSION_MAJOR != 1
// <rclcpp/rclcpp.hpp> already included by compat.h above.
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#else
#include <boost/make_shared.hpp>
#include <tf2_msgs/TFMessage.h>
#endif
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(tf2_msgs, TFMessage)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, TransformStamped)
// Last: only safe now that every message header above is already
// fully parsed -- see const_ptr.h. Needed for the
// tf2_msgs::TFMessage::ConstPtr in transformCallback() below.
#include <jsk_ros1_ros2_compat/const_ptr.h>

std::map<std::string, geometry_msgs::TransformStamped> tf_map;

void transformCallback(const tf2_msgs::TFMessage::ConstPtr& msg){
  std::pair<std::map<std::string, geometry_msgs::TransformStamped>::iterator, bool> ret;
  for(int i=0; i<msg->transforms.size(); i++){
    geometry_msgs::TransformStamped tfs = msg->transforms[i];
    ret = tf_map.insert( std::map<std::string, geometry_msgs::TransformStamped>::value_type(tfs.child_frame_id, tfs) );

    //update value
    if(!ret.second){
      tf_map[tfs.child_frame_id] = tfs;
    }
  }
}

int main(int argc, char** argv)
{
#if ROS_VERSION_MAJOR != 1
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("transform_merger");

  auto pub_ = node->create_publisher<tf2_msgs::TFMessage>("/tf_merged", 1);
  auto sub_ = node->create_subscription<tf2_msgs::TFMessage>(
    "/tf", 100, transformCallback);

  double loop_hz = node->declare_parameter<double>("loop_hz", 1.0);
  RCLCPP_INFO_STREAM(node->get_logger(), "loop_hz:" << loop_hz);

  rclcpp::WallRate rate(loop_hz);
#else
  ros::init(argc, argv, "transform_merger");
  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");

  // pub_ is a shared_ptr here (rather than a plain ros::Publisher, as in
  // the original) purely so that `pub_->publish(...)` below can be
  // shared verbatim with ROS2's rclcpp::Publisher::SharedPtr, with no
  // per-call-site branching needed.
  boost::shared_ptr<ros::Publisher> pub_ = boost::make_shared<ros::Publisher>(
    pnh_.advertise<tf2_msgs::TFMessage> ("/tf_merged", 1));
  ros::Subscriber sub_ =  pnh_.subscribe<tf2_msgs::TFMessage>
    ("/tf", 100, transformCallback);

  double loop_hz;
  pnh_.param("loop_hz", loop_hz, 1.0 );
  ROS_INFO_STREAM("loop_hz:" << loop_hz);

  ros::Rate rate(loop_hz);
#endif

  // set ignore tf
  std::vector< std::string > ignore_tf_vec;

#if ROS_VERSION_MAJOR == 1
  XmlRpc::XmlRpcValue v;
  pnh_.param("transform_config", v, v);
  if(v.hasMember("ignore_tf")){
    XmlRpc::XmlRpcValue ignore_v = v["ignore_tf"];
    ROS_INFO_STREAM("ignore following transform");
    for(int i=0; i< ignore_v.size(); i++){
      ignore_tf_vec.push_back(ignore_v[i]);
      ROS_INFO_STREAM("ignore: " << ignore_v[i]);
    }
  }
#endif
  // Not carried over to the ROS2 code path: the `transform_config:
  // {ignore_tf: [...]}` nested-dict rosparam. ROS2 parameters are flat
  // and typed, with no equivalent to XmlRpcValue's dynamic nested
  // structures; ignore_tf_vec stays empty (nothing filtered) here.

#if ROS_VERSION_MAJOR != 1
  while (rclcpp::ok())
#else
  while (ros::ok())
#endif
    {
      tf2_msgs::TFMessage tf_msg;
      std::map<std::string, geometry_msgs::TransformStamped>::iterator it = tf_map.begin();
      while( it != tf_map.end() )
	{

	  std::vector< std::string >::iterator ignore_it = find( ignore_tf_vec.begin(), ignore_tf_vec.end(), it->second.child_frame_id );
	  if(ignore_it == ignore_tf_vec.end()){
	    tf_msg.transforms.push_back( it->second );
	    }
	  ++it;
	}
      pub_->publish(tf_msg);
      tf_map.clear();
      rate.sleep();
#if ROS_VERSION_MAJOR != 1
      rclcpp::spin_some(node);
#else
      ros::spinOnce();
#endif
    }
}
