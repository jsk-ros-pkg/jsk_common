#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <map>

std::map<std::string, geometry_msgs::TransformStamped> tf_map;

void transformCallback(const tf::tfMessage::ConstPtr& msg){
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
  ros::init(argc, argv, "transform_merger");
  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");
  tf::TransformListener tfl_;

  ros::Publisher pub_ =  pnh_.advertise<tf::tfMessage> ("/tf_merged", 1);
  ros::Subscriber sub_ =  pnh_.subscribe<tf::tfMessage>
    ("/tf", 100, transformCallback);

  double loop_hz;
  pnh_.param("loop_hz", loop_hz, 1.0 );
  ROS_INFO_STREAM("loop_hz:" << loop_hz);

  ros::Rate rate(loop_hz);

  while (ros::ok())
    {
      tf::tfMessage tf_msg;
      std::map<std::string, geometry_msgs::TransformStamped>::iterator it = tf_map.begin();
      while( it != tf_map.end() )
	{
	  tf_msg.transforms.push_back( it->second );
	  ++it;
	}
      pub_.publish(tf_msg);
      tf_map.clear();
      rate.sleep();
      ros::spinOnce();
    }
}
