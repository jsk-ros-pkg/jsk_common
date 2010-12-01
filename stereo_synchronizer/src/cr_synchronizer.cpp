#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class CRSynchronizer {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_, *it_left_, *it_right_, *it_range_;
  image_transport::CameraPublisher pub_left_, pub_right_, pub_range_;
  image_transport::Publisher pub_intent_, pub_confi_;
  image_transport::SubscriberFilter image_sub_left_, image_sub_right_;
  image_transport::SubscriberFilter image_sub_depth_, image_sub_intent_, image_sub_confi_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_left_, info_sub_right_, info_sub_range_;

  message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
						    sensor_msgs::Image, sensor_msgs::CameraInfo,
						    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
						    sensor_msgs::CameraInfo> > sync_;
  std::string left_ns_, right_ns_, range_ns_;
public:
  CRSynchronizer () : nh_(), it_(nh_), sync_(15) {

    std::string left_raw = nh_.resolveName("left_raw");
    std::string right_raw = nh_.resolveName("right_raw");
    std::string range_raw = nh_.resolveName("range_raw");

    image_sub_left_.subscribe(it_, left_raw  + "/image_raw", 8);
    info_sub_left_ .subscribe(nh_, left_raw  + "/camera_info", 8);

    image_sub_right_.subscribe(it_, right_raw + "/image_raw", 8);
    info_sub_right_ .subscribe(nh_, right_raw + "/camera_info", 8);

    image_sub_depth_.subscribe(it_, range_raw + "/distance/image_raw16", 8);
    image_sub_intent_.subscribe(it_, range_raw + "/intensity/image_raw", 8);
    image_sub_confi_.subscribe(it_, range_raw + "/confidence/image_raw", 8);
    info_sub_range_ .subscribe(nh_, range_raw + "/camera_info", 8);

    left_ns_ = nh_.resolveName("left");
    right_ns_ = nh_.resolveName("right");
    range_ns_ = nh_.resolveName("range");
    ros::NodeHandle cam_left_nh(left_ns_);
    ros::NodeHandle cam_right_nh(right_ns_);
    ros::NodeHandle cam_range_nh(range_ns_);

    it_left_ = new image_transport::ImageTransport(cam_left_nh);
    it_right_ = new image_transport::ImageTransport(cam_right_nh);
    it_range_ = new image_transport::ImageTransport(cam_range_nh);

    pub_left_ = it_left_->advertiseCamera("image_raw", 1);
    pub_right_ = it_right_->advertiseCamera("image_raw", 1);
    pub_range_ = it_range_->advertiseCamera("distance/image_raw16", 1);
    pub_intent_ = it_range_->advertise("intensity/image_raw", 1);
    pub_confi_ = it_range_->advertise("confidence/image_raw", 1);

    sync_.connectInput(image_sub_left_, info_sub_left_, image_sub_right_, info_sub_right_,
		       image_sub_depth_, image_sub_intent_, image_sub_confi_, info_sub_range_);
    sync_.registerCallback(boost::bind(&CRSynchronizer::imageCB, this, _1, _2, _3, _4, _5, _6, _7, _8));
  }

  void imageCB(const sensor_msgs::ImageConstPtr& image_left,
	       const sensor_msgs::CameraInfoConstPtr& info_left,
	       const sensor_msgs::ImageConstPtr& image_right,
	       const sensor_msgs::CameraInfoConstPtr& info_right,
	       const sensor_msgs::ImageConstPtr& image_depth,
	       const sensor_msgs::ImageConstPtr& image_intent,
	       const sensor_msgs::ImageConstPtr& image_confi,
	       const sensor_msgs::CameraInfoConstPtr& info_range )  {

    sensor_msgs::Image img_l = *image_left;
    sensor_msgs::CameraInfo info_l = *info_left;
    sensor_msgs::Image img_r = *image_right;
    sensor_msgs::CameraInfo info_r = *info_right;

    pub_left_.publish(img_l, info_l, info_range->header.stamp);
    pub_right_.publish(img_r, info_r, info_range->header.stamp);
    pub_range_.publish(image_depth, info_range);
    pub_intent_.publish(image_intent);
    pub_confi_.publish(image_confi);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cr_synchronizer");

  CRSynchronizer node;

  ros::spin();
  return 0;
}
