#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_model_marker_interface");
  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");
  tf::TransformListener tfl_;

  ros::Publisher pub_ =  pnh_.advertise<geometry_msgs::TransformStamped> ("/specific_transform", 1);

  std::string parent_frame;
  pnh_.param("parent_frame", parent_frame, std::string ("") );

  std::string child_frame;
  pnh_.param("child_frame", child_frame, std::string ("") );

  double loop_hz;
  pnh_.param("loop_hz", loop_hz, 1.0 );

  ROS_INFO_STREAM("parent_frame:" << parent_frame);
  ROS_INFO_STREAM("child_frame:" << child_frame);
  ROS_INFO_STREAM("loop_hz:" << loop_hz);

  ros::Rate rate(loop_hz);

  while (ros::ok())
    {
      geometry_msgs::TransformStamped tf_msg;
      tf::StampedTransform stf;
      try{
      tfl_.lookupTransform(parent_frame, child_frame, ros::Time(0), stf);
      tf::transformStampedTFToMsg(stf, tf_msg);

      pub_.publish(tf_msg);
      }catch(tf::TransformException ex){
	ROS_INFO_STREAM("missing transform: " << parent_frame << " to " << child_frame);
      }

      ros::spinOnce();
      rate.sleep();
    }


}
