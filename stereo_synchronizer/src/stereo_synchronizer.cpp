#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSynchronizer {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_, *it_l_, *it_r_;
  image_transport::CameraPublisher pub_left_, pub_right_;

  image_transport::SubscriberFilter image_sub_l_, image_sub_r_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_l_, info_sub_r_;

  message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
						    sensor_msgs::Image, sensor_msgs::CameraInfo> > sync_;
  std::string left_ns_, right_ns_;
public:
  StereoSynchronizer () : nh_(), it_(nh_), sync_(15) {

    std::string left_raw = nh_.resolveName("left_raw");
    std::string right_raw = nh_.resolveName("right_raw");
    image_sub_l_.subscribe(it_, left_raw  + "/image_raw", 4);
    info_sub_l_ .subscribe(nh_, left_raw  + "/camera_info", 4);
    image_sub_r_.subscribe(it_, right_raw + "/image_raw", 4);
    info_sub_r_ .subscribe(nh_, right_raw + "/camera_info", 4);

    left_ns_ = nh_.resolveName("left");
    right_ns_ = nh_.resolveName("right");
    ros::NodeHandle cam_l_nh(left_ns_);
    ros::NodeHandle cam_r_nh(right_ns_);
    it_l_ = new image_transport::ImageTransport(cam_l_nh);
    it_r_ = new image_transport::ImageTransport(cam_r_nh);
    pub_left_ = it_l_->advertiseCamera("image_raw", 1);
    pub_right_ = it_r_->advertiseCamera("image_raw", 1);

    sync_.connectInput(image_sub_l_, info_sub_l_, image_sub_r_, info_sub_r_);
    sync_.registerCallback(boost::bind(&StereoSynchronizer::imageCB, this, _1, _2, _3, _4));
  }

  void imageCB(const sensor_msgs::ImageConstPtr& image_l,
	       const sensor_msgs::CameraInfoConstPtr& info_l,
	       const sensor_msgs::ImageConstPtr& image_r,
	       const sensor_msgs::CameraInfoConstPtr& info_r)  {

    sensor_msgs::Image img = *image_r;
    sensor_msgs::CameraInfo info = *info_r;

    //img.header.stamp = image_l->header.stamp;
    //info.header.stamp = info_l->header.stamp;

    pub_left_.publish(image_l, info_l);
    pub_right_.publish(img, info, info_l->header.stamp);
    //pub_right_.publish(*image_r, *info_r, info_l->header.stamp);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_synchronizer");

  StereoSynchronizer node;

  ros::spin();
  return 0;
}
